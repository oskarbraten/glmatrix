use num::Float;
use std::ops::{Mul, MulAssign};
use super::{Vec3, Vec4};

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Quat<T: Float> {
    pub elements: [T; 4]
}

impl<T: Float + MulAssign> Quat<T> {
    pub fn new(x: T, y: T, z: T, w: T) -> Self {
        Self {
            elements: [x, y, z, w]
        }
    }
    
    pub fn identity() -> Self {
        Self::new(T::zero(), T::zero(), T::zero(), T::one())
    }
    
    pub fn from_euler(mut x: T, mut y: T, mut z: T) -> Self {
        let half_to_rad = T::from((0.5 * std::f64::consts::PI) / 180.0).unwrap();
        
        x *= half_to_rad;
        y *= half_to_rad;
        z *= half_to_rad;
        
        let sx = x.sin();
        let cx = x.cos();
        let sy = y.sin();
        let cy = y.cos();
        let sz = z.sin();
        let cz = z.cos();
        
        Self::new(sx * cy * cz - cx * sy * sz, cx * sy * cz + sx * cy * sz, cx * cy * sz - sx * sy * cz, cx * cy * sz - sx * sy * cz)
    }
    
    pub fn from_axis_angle(axis: &Vec3<T>, mut rad: T) -> Self {
        rad *= T::from(0.5).unwrap();
        let sin = rad.sin();
        Self::new(axis.elements[0] * sin, axis.elements[1] * sin, axis.elements[2] * sin, rad.cos())
    }
    
    pub fn get_axis_angle(&self) -> (Vec3<T>, T) {
        let acos = self.elements[3].acos();
        let rad = acos * (T::one() + T::one());
        let sin = acos.sin();
        
        if sin > T::zero() {
            (Vec3::new(self.elements[0] / sin, self.elements[1] / sin, self.elements[2] / sin), rad)
        } else {
            (Vec3::new(T::one(), T::one(), T::one()), rad)
        }
    }
    
    pub fn dot(&self, other: &Quat<T>) -> T {
        self.elements[0] * other.elements[0] + self.elements[1] * other.elements[1] + self.elements[2] * other.elements[2] + self.elements[3] * other.elements[3]
    }
    
    pub fn angle(&self, other: &Self) -> T {
        let dot = self.dot(other);
        ((T::one() + T::one()) * dot * dot - T::one()).acos()
    }
    
    pub fn inverse(&self) -> Self {
        let dot = self.dot(self);
        
        if dot == T::zero() {
            Self::new(T::zero(), T::zero(), T::zero(), T::zero())
        } else {
            let inverse_dot = T::one() / dot;
            
            Self::new(-self.elements[0] * inverse_dot, -self.elements[1] * inverse_dot, -self.elements[2] * inverse_dot, -self.elements[3] * inverse_dot,)
        }
    }
    
    pub fn conjugate(&self) -> Self {
        Self::new(-self.elements[0], -self.elements[1], -self.elements[2], self.elements[3])
    }
    
    pub fn length_squared(&self) -> T {
        self.elements[0].powi(2) + self.elements[1].powi(2) + self.elements[2].powi(2) + self.elements[3].powi(2)
    }
    
    pub fn length(&self) -> T {
        self.length_squared().sqrt()
    }
    
    pub fn normalize(&mut self) {
        let mut ls = self.length_squared();
        if ls > T::zero() {
            ls = T::one() / ls.sqrt();
        }
        
        self.elements[0] *= ls;
        self.elements[1] *= ls;
        self.elements[2] *= ls;
        self.elements[3] *= ls;
    }
    
    pub fn normalized(&self) -> Self {
        let mut ls = self.length_squared();
        if ls > T::zero() {
            ls = T::one() / ls.sqrt();
        }
        
        Self::new(self.elements[0] * ls, self.elements[1] * ls, self.elements[2] * ls, self.elements[3] * ls)
    }
}

impl<T: Float + MulAssign> Mul for Quat<T> {
    type Output = Self;
    
    fn mul(self, other: Self) -> Self {
        let [ax, ay, az, aw] = self.elements;
        let [bx, by, bz, bw] = other.elements;

        Self::new(
            ax * bw + aw * bx + ay * bz - az * by,
            ay * bw + aw * by + az * bx - ax * bz,
            az * bw + aw * bz + ax * by - ay * bx,
            aw * bw - ax * bx - ay * by - az * bz
        )
    }
}

impl<T: Float + MulAssign> MulAssign for Quat<T> {
    fn mul_assign(&mut self, other: Self) {
        let [ax, ay, az, aw] = self.elements;
        let [bx, by, bz, bw] = other.elements;
        
        let x = ax * bw + aw * bx + ay * bz - az * by;
        let y = ay * bw + aw * by + az * bx - ax * bz;
        let z = az * bw + aw * bz + ax * by - ay * bx;
        let w = aw * bw - ax * bx - ay * by - az * bz;
        
        self.elements[0] = x;
        self.elements[1] = y;
        self.elements[2] = z;
        self.elements[3] = w;
    }
}

/// Shorthand for v.rotate_quat(...)
impl<T: Float + MulAssign> Mul<Vec3<T>> for Quat<T> {
    type Output = Vec3<T>;
    
    fn mul(self, mut other: Vec3<T>) -> Vec3<T> {
        other.rotate_quat(self);
        other
    }
}

impl<T: Float + MulAssign> Mul<Vec4<T>> for Quat<T> {
    type Output = Vec4<T>;
    
    fn mul(self, mut other: Vec4<T>) -> Vec4<T> {
        other.rotate_quat(self);
        other
    }
}