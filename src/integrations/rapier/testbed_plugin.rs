use crate::math::{Real, Vector};
use crate::object::{BoundaryHandle, FluidHandle};
#[cfg(feature = "dim2")]
use kiss3d::prelude::Vec2;
#[cfg(feature = "dim3")]
use kiss3d::prelude::Vec3;
use kiss3d::{color::Color, window::Window};
use na::Vector3;
use rapier_testbed::{
    egui, harness::Harness, settings::ExampleSettings, GraphicsManager, PhysicsState, Testbed,
    TestbedPlugin,
};

use crate::integrations::rapier::{DfsphParameters, FluidsPipeline};
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

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

const SETTINGS_RENDER_BOUNDARIES: &str = "Fluid render boundaries";
const SETTINGS_RENDERING_MODE: &str = "Fluid rendering mode";
const SETTINGS_MAX_PRESSURE_ITER: &str = "Fluid max pressure iter";
const SETTINGS_MAX_DIVERGENCE_ITER: &str = "Fluid max divergence iter";
const SETTINGS_MAX_DENSITY_ERROR: &str = "Fluid max density error";
const SETTINGS_MAX_DIVERGENCE_ERROR: &str = "Fluid max divergence error";
const SETTINGS_BOUNDARY_FORCE_COEFFICIENT: &str = "Fluid boundary force";

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
    dfsph_parameters: DfsphParameters,
    f2color: HashMap<FluidHandle, Vector3<Real>>,
    boundary2color: HashMap<BoundaryHandle, Vector3<Real>>,
    default_fluid_color: Vector3<Real>,
}

struct SharedFluidsTestbedPlugin(Rc<RefCell<FluidsTestbedPlugin>>);

impl FluidsTestbedPlugin {
    /// Initializes the plugin.
    pub fn new() -> Self {
        Self {
            render_boundary_particles: false,
            fluids_rendering_mode: FluidsRenderingMode::StaticColor,
            step_time: 0.0,
            callbacks: Vec::new(),
            fluids_pipeline: FluidsPipeline::new(0.025, 2.0),
            dfsph_parameters: DfsphParameters::default(),
            f2color: HashMap::new(),
            boundary2color: HashMap::new(),
            default_fluid_color: Vector3::new(0.0, 0.0, 0.5),
        }
    }

    /// Adds a callback to be executed at each frame.
    pub fn add_callback(&mut self, f: impl FnMut(&mut Harness, &mut FluidsPipeline) + 'static) {
        self.callbacks.push(Box::new(f))
    }

    /// Adds this plugin to the Rapier testbed and registers fluid rendering.
    pub fn add_to_testbed(self, testbed: &mut Testbed) {
        let plugin = Rc::new(RefCell::new(self));
        let renderer = Rc::clone(&plugin);

        testbed.add_callback(move |graphics, _physics, _events, _run_state| {
            if let Some(graphics) = graphics {
                let mut renderer = renderer.borrow_mut();

                if let Some(settings) = graphics.settings.as_deref_mut() {
                    renderer.update_from_settings(settings);
                }

                renderer.draw_fluids(graphics.window);
            }
        });

        testbed.add_plugin(SharedFluidsTestbedPlugin(plugin));
    }

    /// Sets the fluids pipeline used by the testbed.
    pub fn set_pipeline(&mut self, fluids_pipeline: FluidsPipeline) {
        self.fluids_pipeline = fluids_pipeline;
        self.fluids_pipeline.liquid_world.counters.enable();
        self.refresh_dfsph_parameters();
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

    fn refresh_dfsph_parameters(&mut self) {
        if let Some(parameters) = self.fluids_pipeline.dfsph_parameters() {
            self.dfsph_parameters = parameters;
        }
    }

    fn rendering_mode_index(&self) -> usize {
        FLUIDS_RENDERING_MAP
            .iter()
            .position(|(_, mode)| *mode == self.fluids_rendering_mode)
            .unwrap_or(0)
    }

    fn get_clamped_u32_setting(
        settings: &mut ExampleSettings,
        key: &'static str,
        default: u32,
        min: u32,
        max: u32,
    ) -> u32 {
        let value = settings
            .get_or_set_u32(key, default.clamp(min, max), min..=max)
            .clamp(min, max);
        settings.set_u32(key, value, min..=max);
        value
    }

    fn update_from_settings(&mut self, settings: &mut ExampleSettings) {
        self.render_boundary_particles =
            settings.get_or_set_bool(SETTINGS_RENDER_BOUNDARIES, self.render_boundary_particles);

        let rendering_options = FLUIDS_RENDERING_MAP
            .iter()
            .map(|(name, _)| (*name).to_string())
            .collect();
        let rendering_mode = settings.get_or_set_string(
            SETTINGS_RENDERING_MODE,
            self.rendering_mode_index(),
            rendering_options,
        );

        if let Some((_, mode)) = FLUIDS_RENDERING_MAP.get(rendering_mode) {
            self.fluids_rendering_mode = *mode;
        }

        let max_pressure_iter = Self::get_clamped_u32_setting(
            settings,
            SETTINGS_MAX_PRESSURE_ITER,
            self.dfsph_parameters.max_pressure_iter as u32,
            1,
            80,
        ) as usize;
        let max_divergence_iter = Self::get_clamped_u32_setting(
            settings,
            SETTINGS_MAX_DIVERGENCE_ITER,
            self.dfsph_parameters.max_divergence_iter as u32,
            1,
            80,
        ) as usize;
        let max_density_error = settings.get_or_set_f32(
            SETTINGS_MAX_DENSITY_ERROR,
            self.dfsph_parameters.max_density_error as f32,
            0.0..=0.5,
        );
        let max_divergence_error = settings.get_or_set_f32(
            SETTINGS_MAX_DIVERGENCE_ERROR,
            self.dfsph_parameters.max_divergence_error as f32,
            0.0..=2.0,
        );

        let boundary_force = settings.get_or_set_f32(
            SETTINGS_BOUNDARY_FORCE_COEFFICIENT,
            self.fluids_pipeline.liquid_world.boundary_force_coefficient as f32,
            0.0..=1.0,
        );
        self.fluids_pipeline.liquid_world.boundary_force_coefficient =
            na::convert::<_, Real>(boundary_force);

        let dfsph_parameters = DfsphParameters {
            min_pressure_iter: self.dfsph_parameters.min_pressure_iter,
            max_pressure_iter,
            max_density_error: na::convert::<_, Real>(max_density_error),
            min_divergence_iter: self.dfsph_parameters.min_divergence_iter,
            max_divergence_iter,
            max_divergence_error: na::convert::<_, Real>(max_divergence_error),
        };

        if dfsph_parameters != self.dfsph_parameters {
            self.dfsph_parameters = dfsph_parameters;
            let _ = self.fluids_pipeline.set_dfsph_parameters(dfsph_parameters);
        }
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

    fn draw_fluids(&self, window: &mut Window) {
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
        self.draw_fluids(window);
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

impl TestbedPlugin for SharedFluidsTestbedPlugin {
    fn init_plugin(&mut self) {
        self.0.borrow_mut().init_plugin();
    }

    fn init_graphics(
        &mut self,
        graphics: &mut GraphicsManager,
        window: &mut Window,
        harness: &mut Harness,
    ) {
        self.0.borrow_mut().init_graphics(graphics, window, harness);
    }

    fn clear_graphics(&mut self, graphics: &mut GraphicsManager, window: &mut Window) {
        self.0.borrow_mut().clear_graphics(graphics, window);
    }

    fn run_callbacks(&mut self, harness: &mut Harness) {
        self.0.borrow_mut().run_callbacks(harness);
    }

    fn step(&mut self, physics: &mut PhysicsState) {
        self.0.borrow_mut().step(physics);
    }

    fn draw(&mut self, graphics: &mut GraphicsManager, window: &mut Window, harness: &mut Harness) {
        self.0.borrow_mut().draw(graphics, window, harness);
    }

    fn update_ui(
        &mut self,
        ui_context: &egui::Context,
        harness: &mut Harness,
        graphics: &mut GraphicsManager,
        window: &mut Window,
    ) {
        self.0
            .borrow_mut()
            .update_ui(ui_context, harness, graphics, window);
    }

    fn profiling_string(&self) -> String {
        self.0.borrow().profiling_string()
    }
}
