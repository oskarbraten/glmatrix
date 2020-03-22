use std::ops::{Add, AddAssign, Sub, SubAssign, Mul, MulAssign, Div, DivAssign};
use super::Vec3;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Quat {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32
}

impl Quat {
    pub fn new(x: f32, y: f32, z: f32, w: f32) -> Self {
        Self {
            x,
            y,
            z,
            w
        }
    }

    pub fn identity() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0
        }
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

        Self {
            x: sx * cy * cz - cx * sy * sz,
            y: cx * sy * cz + sx * cy * sz,
            z: cx * cy * sz - sx * sy * cz,
            w: cx * cy * sz - sx * sy * cz
        }
    }

    pub fn from_axis_angle(axis: &Vec3, mut rad: f32) -> Self {
        rad *= 0.5;
        let sin = rad.sin();
        Self {
            x: axis.x * sin,
            y: axis.y * sin,
            z: axis.z * sin,
            w: rad.cos()
        }
    }

    pub fn get_axis_angle(&self) -> (Vec3, f32) {
        let acos = self.w.acos();
        let rad = acos * 2.0;
        let sin = acos.sin();

        if sin > 0.0 {
            (Vec3::new(self.x / sin, self.y / sin, self.z / sin), rad)
        } else {
            (Vec3::new(1.0, 0.0, 0.0), rad)
        }
    }

    pub fn dot(&self, other: &Quat) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z + self.w * other.w
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

            Self {
                x: -self.x * inverse_dot,
                y: -self.y * inverse_dot,
                z: -self.z * inverse_dot,
                w: -self.w * inverse_dot,
            }
        }
    }

    pub fn conjugate(&self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
            w: self.w,
        }
    }
}

impl Mul for Quat {
    type Output = Self;

    fn mul(self, other: Self) -> Self {
        Self {
            x: self.x * other.w + self.w * other.x + self.y * other.z - self.z * other.y,
            y: self.y * other.w + self.w * other.y + self.z * other.x - self.x * other.z,
            z: self.z * other.w + self.w * other.z + self.x * other.y - self.y * other.x,
            w: self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
        }
    }
}

impl MulAssign for Quat {
    fn mul_assign(&mut self, other: Self) {
        let x = self.x * other.w + self.w * other.x + self.y * other.z - self.z * other.y;
        let y = self.y * other.w + self.w * other.y + self.z * other.x - self.x * other.z;
        let z = self.z * other.w + self.w * other.z + self.x * other.y - self.y * other.x;
        let w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z;

        self.x = x;
        self.y = y;
        self.z = z;
        self.w = w;
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