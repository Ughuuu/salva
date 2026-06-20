use crate::math::{Real, Vector};
use crate::object::{BoundaryHandle, FluidHandle};
#[cfg(feature = "dim2")]
use kiss3d::prelude::Vec2;
#[cfg(feature = "dim3")]
use kiss3d::prelude::Vec3;
use kiss3d::{color::Color, window::Window};
use na::Vector3;
use rapier_testbed::{egui, harness::Harness, GraphicsManager, PhysicsState, TestbedPlugin};

use crate::integrations::rapier::FluidsPipeline;
use std::collections::HashMap;

pub const FLUIDS_RENDERING_MAP: [(&str, FluidsRenderingMode); 3] = [
    ("Static", FluidsRenderingMode::StaticColor),
    (
        "Velocity Color",
        FluidsRenderingMode::VelocityColor {
            min: 0.0,
            max: 50.0,
        },
    ),
    (
        "Velocity Arrows",
        FluidsRenderingMode::VelocityArrows {
            min: 0.0,
            max: 50.0,
        },
    ),
];

/// How the fluids should be rendered by the testbed.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum FluidsRenderingMode {
    /// Use a plain color.
    StaticColor,
    /// Use a red taint the closer to `max` the velocity is.
    VelocityColor {
        /// Fluids with a velocity smaller than this will not have any red taint.
        min: Real,
        /// Fluids with a velocity greater than this will be completely red.
        max: Real,
    },
    /// Show particles as arrows indicating the velocity.
    VelocityArrows {
        /// Fluids with a velocity smaller than this will not have any red taint.
        min: Real,
        /// Fluids with a velocity greater than this will be completely red.
        max: Real,
    },
}

/// A user-defined callback executed at each frame.
pub type FluidCallback = Box<dyn FnMut(&mut Harness, &mut FluidsPipeline)>;

/// A plugin for stepping fluids inside the Rapier testbed.
pub struct FluidsTestbedPlugin {
    /// Whether to render the boundary particles.
    pub render_boundary_particles: bool,
    /// Rendering mode of fluid particles.
    pub fluids_rendering_mode: FluidsRenderingMode,
    callbacks: Vec<FluidCallback>,
    step_time: f64,
    fluids_pipeline: FluidsPipeline,
    f2color: HashMap<FluidHandle, Vector3<Real>>,
    boundary2color: HashMap<BoundaryHandle, Vector3<Real>>,
    default_fluid_color: Vector3<Real>,
}

impl FluidsTestbedPlugin {
    /// Initializes the plugin.
    pub fn new() -> Self {
        Self {
            render_boundary_particles: false,
            fluids_rendering_mode: FluidsRenderingMode::StaticColor,
            step_time: 0.0,
            callbacks: Vec::new(),
            fluids_pipeline: FluidsPipeline::new(0.025, 2.0),
            f2color: HashMap::new(),
            boundary2color: HashMap::new(),
            default_fluid_color: Vector3::new(0.0, 0.0, 0.5),
        }
    }

    /// Adds a callback to be executed at each frame.
    pub fn add_callback(&mut self, f: impl FnMut(&mut Harness, &mut FluidsPipeline) + 'static) {
        self.callbacks.push(Box::new(f))
    }

    /// Sets the fluids pipeline used by the testbed.
    pub fn set_pipeline(&mut self, fluids_pipeline: FluidsPipeline) {
        self.fluids_pipeline = fluids_pipeline;
        self.fluids_pipeline.liquid_world.counters.enable();
    }

    /// Sets the color used to render the specified fluid.
    pub fn set_fluid_color(&mut self, fluid: FluidHandle, color: Vector3<Real>) {
        let _ = self.f2color.insert(fluid, color);
    }

    /// Sets the way fluids are rendered.
    pub fn set_fluid_rendering_mode(&mut self, mode: FluidsRenderingMode) {
        self.fluids_rendering_mode = mode;
    }

    /// Enables the rendering of boundary particles.
    pub fn enable_boundary_particles_rendering(&mut self, enabled: bool) {
        self.render_boundary_particles = enabled;
    }

    fn color(color: Vector3<Real>) -> Color {
        Color::new(color.x as f32, color.y as f32, color.z as f32, 1.0)
    }

    fn fluid_color(&self, handle: FluidHandle, velocity: &Vector<Real>) -> Color {
        let base = *self
            .f2color
            .get(&handle)
            .unwrap_or(&self.default_fluid_color);

        match self.fluids_rendering_mode {
            FluidsRenderingMode::StaticColor => Self::color(base),
            FluidsRenderingMode::VelocityColor { min, max }
            | FluidsRenderingMode::VelocityArrows { min, max } => {
                Self::color(Self::velocity_tinted_color(base, velocity, min, max))
            }
        }
    }

    fn velocity_tinted_color(
        base: Vector3<Real>,
        velocity: &Vector<Real>,
        min: Real,
        max: Real,
    ) -> Vector3<Real> {
        let range = max - min;

        if range <= na::zero() {
            return base;
        }

        let factor = ((velocity.norm() - min) / range)
            .max(na::zero())
            .min(na::one());
        let inv_factor = na::one::<Real>() - factor;

        Vector3::new(
            base.x * inv_factor + factor,
            base.y * inv_factor,
            base.z * inv_factor,
        )
    }

    fn particle_size(radius: Real) -> f32 {
        (radius as f32 * 300.0).clamp(2.5, 8.0)
    }

    #[cfg(feature = "dim2")]
    fn point(point: &Vector<Real>) -> Vec2 {
        Vec2::new(point.x as f32, point.y as f32)
    }

    #[cfg(feature = "dim3")]
    fn point(point: &Vector<Real>) -> Vec3 {
        Vec3::new(point.x as f32, point.y as f32, point.z as f32)
    }

    #[cfg(feature = "dim2")]
    fn draw_particle(window: &mut Window, point: &Vector<Real>, color: Color, size: f32) {
        window.draw_point_2d(Self::point(point), color, size);
    }

    #[cfg(feature = "dim3")]
    fn draw_particle(window: &mut Window, point: &Vector<Real>, color: Color, size: f32) {
        window.draw_point(Self::point(point), color, size);
    }

    #[cfg(feature = "dim2")]
    fn draw_velocity(
        window: &mut Window,
        point: &Vector<Real>,
        velocity: &Vector<Real>,
        color: Color,
    ) {
        let end = *point + *velocity * na::convert::<_, Real>(0.02);
        window.draw_line_2d(Self::point(point), Self::point(&end), color, 1.5);
    }

    #[cfg(feature = "dim3")]
    fn draw_velocity(
        window: &mut Window,
        point: &Vector<Real>,
        velocity: &Vector<Real>,
        color: Color,
    ) {
        let end = *point + *velocity * na::convert::<_, Real>(0.02);
        window.draw_line(Self::point(point), Self::point(&end), color, 1.5, false);
    }
}

impl TestbedPlugin for FluidsTestbedPlugin {
    fn init_plugin(&mut self) {}

    fn init_graphics(
        &mut self,
        _graphics: &mut GraphicsManager,
        _window: &mut Window,
        _harness: &mut Harness,
    ) {
    }

    fn clear_graphics(&mut self, _graphics: &mut GraphicsManager, _window: &mut Window) {}

    fn run_callbacks(&mut self, harness: &mut Harness) {
        for f in &mut self.callbacks {
            f(harness, &mut self.fluids_pipeline)
        }
    }

    fn step(&mut self, physics: &mut PhysicsState) {
        let dt = physics.integration_parameters.dt;
        self.fluids_pipeline.step(
            &physics.gravity,
            dt,
            &physics.colliders,
            &mut physics.bodies,
        );
    }

    fn draw(
        &mut self,
        _graphics: &mut GraphicsManager,
        window: &mut Window,
        _harness: &mut Harness,
    ) {
        let draw_velocities = matches!(
            self.fluids_rendering_mode,
            FluidsRenderingMode::VelocityArrows { .. }
        );

        for (handle, fluid) in self.fluids_pipeline.liquid_world.fluids().iter() {
            let size = Self::particle_size(fluid.particle_radius());

            for (point, velocity) in fluid.positions.iter().zip(fluid.velocities.iter()) {
                let color = self.fluid_color(handle, velocity);
                Self::draw_particle(window, point, color, size);

                if draw_velocities && velocity.norm_squared() > na::zero() {
                    Self::draw_velocity(window, point, velocity, color);
                }
            }
        }

        if self.render_boundary_particles {
            let default_color = Vector3::repeat(na::convert::<_, Real>(0.5));
            let size = Self::particle_size(self.fluids_pipeline.liquid_world.particle_radius());

            for (handle, boundary) in self.fluids_pipeline.liquid_world.boundaries().iter() {
                let color =
                    Self::color(*self.boundary2color.get(&handle).unwrap_or(&default_color));

                for point in &boundary.positions {
                    Self::draw_particle(window, point, color, size);
                }
            }
        }
    }

    fn update_ui(
        &mut self,
        ui_context: &egui::Context,
        _harness: &mut Harness,
        _graphics: &mut GraphicsManager,
        _window: &mut Window,
    ) {
        let _ = egui::Window::new("Fluids").show(ui_context, |ui| {
            let _ = ui.checkbox(
                &mut self.render_boundary_particles,
                "Render boundary particles",
            );

            let selected = FLUIDS_RENDERING_MAP
                .iter()
                .find_map(|(name, mode)| (*mode == self.fluids_rendering_mode).then_some(*name))
                .unwrap_or("Custom");

            let _ = egui::ComboBox::from_label("Rendering mode")
                .selected_text(selected)
                .show_ui(ui, |ui| {
                    for (name, mode) in FLUIDS_RENDERING_MAP {
                        let _ = ui.selectable_value(&mut self.fluids_rendering_mode, mode, name);
                    }
                });
        });
    }

    fn profiling_string(&self) -> String {
        format!("Fluids: {:.2}ms", self.step_time)
    }
}
