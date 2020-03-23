use std::ops::{Mul, MulAssign};
use super::{Vec3, Vec4};

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Quat {
    pub elements: [f32; 4]
}

impl Quat {
    pub const fn new(x: f32, y: f32, z: f32, w: f32) -> Self {
        Self {
            elements: [x, y, z, w]
        }
    }
    
    pub const fn identity() -> Self {
        Self::new(0.0, 0.0, 0.0, 1.0)
    }
    
    pub fn from_euler(mut x: f32, mut y: f32, mut z: f32) -> Self {
        let half_to_rad = (0.5 * std::f32::consts::PI) / 180.0;
        
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
    
    pub fn from_axis_angle(axis: &Vec3, mut rad: f32) -> Self {
        rad *= 0.5;
        let sin = rad.sin();
        Self::new(axis.elements[0] * sin, axis.elements[1] * sin, axis.elements[2] * sin, rad.cos())
    }
    
    pub fn get_axis_angle(&self) -> (Vec3, f32) {
        let acos = self.elements[3].acos();
        let rad = acos * 2.0;
        let sin = acos.sin();
        
        if sin > 0.0 {
            (Vec3::new(self.elements[0] / sin, self.elements[1] / sin, self.elements[2] / sin), rad)
        } else {
            (Vec3::new(1.0, 0.0, 0.0), rad)
        }
    }
    
    pub fn dot(&self, other: &Quat) -> f32 {
        self.elements[0] * other.elements[0] + self.elements[1] * other.elements[1] + self.elements[2] * other.elements[2] + self.elements[3] * other.elements[3]
    }
    
    pub fn angle(&self, other: &Self) -> f32 {
        let dot = self.dot(other);
        (2.0 * dot * dot - 1.0).acos()
    }
    
    pub fn inverse(&self) -> Self {
        let dot = self.dot(self);
        
        if dot == 0.0 {
            Self::new(0.0, 0.0, 0.0, 0.0)
        } else {
            let inverse_dot = 1.0 / dot;
            
            Self::new(-self.elements[0] * inverse_dot, -self.elements[1] * inverse_dot, -self.elements[2] * inverse_dot, -self.elements[3] * inverse_dot,)
        }
    }
    
    pub fn conjugate(&self) -> Self {
        Self::new(-self.elements[0], -self.elements[1], -self.elements[2], self.elements[3])
    }
    
    pub fn length_squared(&self) -> f32 {
        self.elements[0].powi(2) + self.elements[1].powi(2) + self.elements[2].powi(2) + self.elements[3].powi(2)
    }
    
    pub fn length(&self) -> f32 {
        self.length_squared().sqrt()
    }
    
    pub fn normalize(&mut self) {
        let mut ls = self.length_squared();
        if ls > 0.0 {
            ls = 1.0 / ls.sqrt();
        }
        
        self.elements[0] *= ls;
        self.elements[1] *= ls;
        self.elements[2] *= ls;
        self.elements[3] *= ls;
    }
    
    pub fn normalized(&self) -> Self {
        let mut ls = self.length_squared();
        if ls > 0.0 {
            ls = 1.0 / ls.sqrt();
        }
        
        Self::new(self.elements[0] * ls, self.elements[1] * ls, self.elements[2] * ls, self.elements[3] * ls)
    }
}

impl Mul for Quat {
    type Output = Self;
    
    fn mul(self, other: Self) -> Self {
        let [ax, ay, az, aw] = &self.elements;
        let [bx, by, bz, bw] = &other.elements;

        Self::new(
            ax * bw + aw * bx + ay * bz - az * by,
            ay * bw + aw * by + az * bx - ax * bz,
            az * bw + aw * bz + ax * by - ay * bx,
            aw * bw - ax * bx - ay * by - az * bz
        )
    }
}

impl MulAssign for Quat {
    fn mul_assign(&mut self, other: Self) {
        let [ax, ay, az, aw] = &self.elements;
        let [bx, by, bz, bw] = &other.elements;
        
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
impl Mul<Vec3> for Quat {
    type Output = Vec3;
    
    fn mul(self, mut other: Vec3) -> Vec3 {
        other.rotate_quat(self);
        other
    }
}

impl Mul<Vec4> for Quat {
    type Output = Vec4;
    
    fn mul(self, mut other: Vec4) -> Vec4 {
        other.rotate_quat(self);
        other
    }
}