use std::marker::PhantomData;

#[cfg(feature = "parallel")]
use rayon::prelude::*;

use approx::AbsDiffEq;

use crate::geometry::{self, ParticlesContacts};
use crate::kernel::{CubicSplineKernel, Kernel};
use crate::math::{Matrix, Point, Real, RotationMatrix, SpatialVector, Vector};
use crate::object::{Boundary, Fluid};
use crate::solver::NonPressureForce;
use crate::TimestepManager;

fn elasticity_coefficients(young_modulus: Real, poisson_ratio: Real) -> (Real, Real, Real) {
    let _1 = na::one::<Real>();
    let _2: Real = na::convert::<_, Real>(2.0);

    let d0 =
        (young_modulus * (_1 - poisson_ratio)) / ((_1 + poisson_ratio) * (_1 - _2 * poisson_ratio));
    let d1 = (young_modulus * poisson_ratio) / ((_1 + poisson_ratio) * (_1 - _2 * poisson_ratio));
    let d2 = (young_modulus * (_1 - _2 * poisson_ratio))
        / (_2 * (_1 + poisson_ratio) * (_1 - _2 * poisson_ratio));
    (d0, d1, d2)
}

fn sym_mat_mul_vec(mat: &SpatialVector<Real>, v: &Vector<Real>) -> Vector<Real> {
    #[cfg(feature = "dim2")]
    return Vector::new(mat.x * v.x + mat.z * v.y, mat.z * v.x + mat.y * v.y);

    #[cfg(feature = "dim3")]
    return Vector::new(
        mat.x * v.x + mat.w * v.y + mat.a * v.z,
        mat.w * v.x + mat.y * v.y + mat.b * v.z,
        mat.a * v.x + mat.b * v.y + mat.z * v.z,
    );
}

// https://cg.informatik.uni-freiburg.de/publications/2009_NP_corotatedSPH.pdf
/// Elasticity based on the method from Becker et al. 2009.
pub struct Becker2009Elasticity<
    KernelDensity: Kernel = CubicSplineKernel,
    KernelGradient: Kernel = CubicSplineKernel,
> {
    d0: Real,
    d1: Real,
    d2: Real,
    nonlinear_strain: bool,
    volumes0: Vec<Real>,
    positions0: Vec<Point<Real>>,
    contacts0: ParticlesContacts,
    rotations: Vec<RotationMatrix<Real>>,
    deformation_gradient_tr: Vec<Matrix<Real>>,
    stress: Vec<SpatialVector<Real>>,
    phantom: PhantomData<(KernelDensity, KernelGradient)>,
}

impl<KernelDensity: Kernel, KernelGradient: Kernel>
    Becker2009Elasticity<KernelDensity, KernelGradient>
{
    /// Initialize elasticity from its young modulus and poisson ration.
    ///
    /// If `nonlinear_strain` is `true`, the nonlinear version of the strain tensor is used.
    /// This allows a more realistic simulation of large deformation. However this is slightly more
    /// computationally intensive.
    pub fn new(young_modulus: Real, poisson_ratio: Real, nonlinear_strain: bool) -> Self {
        let (d0, d1, d2) = elasticity_coefficients(young_modulus, poisson_ratio);

        Self {
            d0,
            d1,
            d2,
            nonlinear_strain,
            volumes0: Vec::new(),
            positions0: Vec::new(),
            contacts0: ParticlesContacts::new(),
            rotations: Vec::new(),
            deformation_gradient_tr: Vec::new(),
            stress: Vec::new(),
            phantom: PhantomData,
        }
    }

    fn init(&mut self, kernel_radius: Real, fluid: &Fluid) {
        let nparticles = fluid.positions.len();

        if self.positions0.len() != nparticles {
            self.positions0 = fluid.positions.clone();
            self.volumes0.resize(nparticles, na::zero::<Real>());
            self.rotations
                .resize(nparticles, RotationMatrix::identity());
            self.deformation_gradient_tr
                .resize(nparticles, Matrix::identity());
            self.stress.resize(nparticles, SpatialVector::zeros());
            geometry::compute_self_contacts(kernel_radius, fluid, &mut self.contacts0);

            for contacts in self.contacts0.contacts_mut() {
                for c in contacts.get_mut().unwrap() {
                    let p1 = &self.positions0[c.i];
                    let p2 = &self.positions0[c.j];
                    c.weight = KernelDensity::points_apply(p1, p2, kernel_radius);
                    c.gradient = KernelGradient::points_apply_diff1(p1, p2, kernel_radius);

                    self.volumes0[c.i] += fluid.particle_mass(c.j) * c.weight;
                    self.volumes0[c.j] += fluid.particle_mass(c.i) * c.weight;
                }
            }

            for i in 0..nparticles {
                self.volumes0[i] = fluid.particle_mass(i) / self.volumes0[i];
            }
        }
    }

    fn compute_rotations(&mut self, _kernel_radius: Real, fluid: &Fluid) {
        let _2: Real = na::convert::<_, Real>(2.0f64);

        let contacts0 = &self.contacts0;
        let positions0 = &self.positions0;

        par_iter_mut!(&mut self.rotations)
            .enumerate()
            .for_each(|(i, rotation)| {
                let mut a_pq = Matrix::zeros();

                for c in contacts0.particle_contacts(i).read().unwrap().iter() {
                    let p_ji = fluid.positions[c.j] - fluid.positions[c.i];
                    let p0_ji = positions0[c.j] - positions0[c.i];
                    let coeff = c.weight * fluid.particle_mass(c.j);
                    a_pq += p_ji * (p0_ji * coeff).transpose();
                }

                // Extract the rotation matrix.
                *rotation =
                    RotationMatrix::from_matrix_eps(&a_pq, Real::default_epsilon(), 20, *rotation);
            })
    }

    fn compute_stresses(&mut self, _kernel_radius: Real, fluid: &Fluid) {
        let _2: Real = na::convert::<_, Real>(2.0f64);
        let _0_5: Real = na::convert::<_, Real>(0.564);

        let contacts0 = &self.contacts0;
        let rotations = &self.rotations;
        let positions0 = &self.positions0;

        // let _0 = na::zero::<Real>();
        // let c = Matrix::new(
        //     d0, d1, d1, _0, _0, _0,
        //     d1, d0, d1, _0, _0, _0,
        //     d1, d1, d0, _0, _0, _0,
        //     _0, _0, _0, d2, _0, _0,
        //     _0, _0, _0, _0, d2, _0,
        //     _0, _0, _0, _0, _0, d2,
        // );
        #[rustfmt::skip]
            #[cfg(feature = "dim3")]
            let c_top_left = Matrix::new(
            self.d0, self.d1, self.d1,
            self.d1, self.d0, self.d1,
            self.d1, self.d1, self.d0,
        );
        #[rustfmt::skip]
            #[cfg(feature = "dim2")]
            let c_top_left = Matrix::new(
            self.d0, self.d1,
            self.d1, self.d0,
        );
        let d2 = self.d2;

        let nonlinear_strain = self.nonlinear_strain;
        let volumes0 = &self.volumes0;

        par_iter_mut!(&mut self.deformation_gradient_tr)
            .zip(&mut self.stress)
            .enumerate()
            .for_each(|(i, (deformation_grad_tr, stress))| {
                let mut grad_tr = Matrix::zeros();

                for c in contacts0.particle_contacts(i).read().unwrap().iter() {
                    let p_ji = fluid.positions[c.j] - fluid.positions[c.i];
                    let p0_ji = positions0[c.j] - positions0[c.i];
                    let u_ji = rotations[c.i].inverse_transform_vector(&(p_ji)) - p0_ji;
                    grad_tr += (c.gradient * volumes0[c.j]) * u_ji.transpose();
                }

                *deformation_grad_tr = grad_tr;

                #[cfg(feature = "dim3")]
                {
                    if nonlinear_strain {
                        let j = grad_tr + Matrix::identity();
                        let jjt = j * j.transpose();

                        let stress012 = c_top_left
                            * Vector::new(
                                jjt.m11 - na::one::<Real>(),
                                jjt.m22 - na::one::<Real>(),
                                jjt.m33 - na::one::<Real>(),
                            )
                            * _0_5;
                        *stress = SpatialVector::new(
                            stress012.x,
                            stress012.y,
                            stress012.z,
                            jjt.m21 * _0_5 * d2,
                            jjt.m31 * _0_5 * d2,
                            jjt.m32 * _0_5 * d2,
                        );
                    } else {
                        // let strain = Vector::new(
                        //     grad_tr.m11,
                        //     grad_tr.m22,
                        //     grad_tr.m33,
                        //     (grad_tr.m21 + grad_tr.m12) * _0_5,
                        //     (grad_tr.m31 + grad_tr.m13) * _0_5,
                        //     (grad_tr.m23 + grad_tr.m32) * _0_5,
                        // );

                        let stress012 =
                            c_top_left * Vector::new(grad_tr.m11, grad_tr.m22, grad_tr.m33);
                        *stress = SpatialVector::new(
                            stress012.x,
                            stress012.y,
                            stress012.z,
                            (grad_tr.m21 + grad_tr.m12) * _0_5 * d2,
                            (grad_tr.m31 + grad_tr.m13) * _0_5 * d2,
                            (grad_tr.m23 + grad_tr.m32) * _0_5 * d2,
                        );
                    }
                }

                #[cfg(feature = "dim2")]
                {
                    if nonlinear_strain {
                        let j = grad_tr + Matrix::identity();
                        let jjt = j * j.transpose();

                        let stress01 = c_top_left
                            * Vector::new(jjt.m11 - na::one::<Real>(), jjt.m22 - na::one::<Real>())
                            * _0_5;
                        *stress = SpatialVector::new(stress01.x, stress01.y, jjt.m21 * _0_5 * d2);
                    } else {
                        // let strain = Vector::new(
                        //     grad_tr.m11,
                        //     grad_tr.m22,
                        //     grad_tr.m33,
                        //     (grad_tr.m21 + grad_tr.m12) * _0_5,
                        //     (grad_tr.m31 + grad_tr.m13) * _0_5,
                        //     (grad_tr.m23 + grad_tr.m32) * _0_5,
                        // );

                        let stress01 = c_top_left * Vector::new(grad_tr.m11, grad_tr.m22);
                        *stress = SpatialVector::new(
                            stress01.x,
                            stress01.y,
                            (grad_tr.m21 + grad_tr.m12) * _0_5 * d2,
                        );
                    }
                }
            })
    }
}

impl<KernelDensity: Kernel, KernelGradient: Kernel> NonPressureForce
    for Becker2009Elasticity<KernelDensity, KernelGradient>
{
    fn solve(
        &mut self,
        _timestep: &TimestepManager,
        kernel_radius: Real,
        _fluid_fluid_contacts: &ParticlesContacts,
        _fluid_boundaries_contacts: &ParticlesContacts,
        fluid: &mut Fluid,
        _boundaries: &[Boundary],
        _densities: &[Real],
    ) {
        self.init(kernel_radius, fluid);

        let _0_5: Real = na::convert::<_, Real>(0.5f64);
        self.compute_rotations(kernel_radius, fluid);
        self.compute_stresses(kernel_radius, fluid);

        // Compute and apply forces.
        let contacts0 = &self.contacts0;
        let volumes0 = &self.volumes0;
        let deformation_gradient_tr = &self.deformation_gradient_tr;
        let rotations = &self.rotations;
        let stress = &self.stress;
        let volumes = &fluid.volumes;
        let density0 = fluid.density0;

        if self.nonlinear_strain {
            par_iter_mut!(fluid.accelerations)
                .enumerate()
                .for_each(|(i, acceleration)| {
                    for c in contacts0.particle_contacts(i).read().unwrap().iter() {
                        let mut force = Vector::zeros();

                        let grad_tr_i = &deformation_gradient_tr[c.i];
                        let d_ij = c.gradient * volumes0[c.j];
                        let sigma_d_ij = sym_mat_mul_vec(&stress[c.i], &d_ij);
                        let f_ji = (sigma_d_ij + grad_tr_i * sigma_d_ij) * -volumes0[c.i];

                        let grad_tr_j = &deformation_gradient_tr[c.j];
                        let d_ji = c.gradient * (-volumes0[c.i]);
                        let sigma_d_ji = sym_mat_mul_vec(&stress[c.j], &d_ji);
                        let f_ij = (sigma_d_ji + grad_tr_j * sigma_d_ji) * -volumes0[c.j];

                        force += (rotations[c.j] * f_ij - (rotations[c.i] * f_ji)) * _0_5;

                        *acceleration += force / (volumes[i] * density0);
                    }
                })
        } else {
            par_iter_mut!(fluid.accelerations)
                .enumerate()
                .for_each(|(i, acceleration)| {
                    for c in contacts0.particle_contacts(i).read().unwrap().iter() {
                        let mut force = Vector::zeros();

                        let d_ij = c.gradient * volumes0[c.j];
                        let f_ji = sym_mat_mul_vec(&stress[c.i], &d_ij) * -volumes0[c.i];

                        let d_ji = c.gradient * (-volumes0[c.i]);
                        let f_ij = sym_mat_mul_vec(&stress[c.j], &d_ji) * -volumes0[c.j];

                        force += (rotations[c.j] * f_ij - (rotations[c.i] * f_ji)) * _0_5;

                        *acceleration += force / (volumes[i] * density0);
                    }
                })
        }
    }

    fn apply_permutation(&mut self, permutation: &[usize]) {
        self.volumes0 = crate::z_order::apply_permutation(permutation, &self.volumes0);
        self.positions0 = crate::z_order::apply_permutation(permutation, &self.positions0);
        self.rotations = crate::z_order::apply_permutation(permutation, &self.rotations);
        self.contacts0.apply_permutation(permutation);
    }
}
