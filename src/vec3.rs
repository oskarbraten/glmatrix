use std::ops::{Add, AddAssign, Sub, SubAssign, Mul, MulAssign, Div, DivAssign};
use super::Quat;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32
}

impl Vec3 {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self {
            x,
            y,
            z
        }
    }

    pub fn zero() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0
        }
    }

    pub fn one() -> Self {
        Self {
            x: 1.0,
            y: 1.0,
            z: 1.0
        }
    }

    pub fn length_squared(&self) -> f32 {
        self.x.powi(2) + self.y.powi(2) + self.z.powi(2)
    }

    pub fn length(&self) -> f32 {
        self.length_squared().sqrt()
    }

    pub fn magnitude(&self) -> f32 {
        self.length()
    }

    pub fn dist_squared(&self, other: &Self) -> f32 {
        (other.x - self.x).powi(2) + (other.y - self.y).powi(2) + (other.z - self.z).powi(2)
    }

    pub fn dist(&self, other: &Self) -> f32 {
        self.dist_squared(other).sqrt()
    }

    pub fn invert(&mut self) {
        self.x = 1.0 / self.x;
        self.y = 1.0 / self.y;
        self.z = 1.0 / self.z;
    }

    pub fn inverse(&self) -> Self {
        Self {
            x: 1.0 / self.x,
            y: 1.0 / self.y,
            z: 1.0 / self.z,
        }
    }

    pub fn normalize(&mut self) {
        let mut ls = self.length_squared();
        if ls > 0.0 {
            ls = 1.0 / ls.sqrt();
        }

        self.x *= ls;
        self.y *= ls;
        self.z *= ls;
    }

    pub fn normalized(&self) -> Self {
        let mut ls = self.length_squared();
        if ls > 0.0 {
            ls = 1.0 / ls.sqrt();
        }

        Self {
            x: self.x * ls,
            y: self.y * ls,
            z: self.z * ls,
        }
    }

    pub fn dot(&self, other: &Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn cross(&self, other: &Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x
        }
    }

    pub fn lerp(&self, other: &Self, t: f32) -> Self {
        Self {
            x: self.x + t * (other.x - self.x),
            y: self.y + t * (other.y - self.y),
            z: self.z + t * (other.z - self.z),
        }
    }

    pub fn rotate_quat(&mut self, quat: Quat) {
        let mut uvx = quat.y * self.z - quat.z * self.y;
        let mut uvy = quat.z * self.x - quat.x * self.z;
        let mut uvz = quat.x * self.y - quat.y * self.x;

        let uuvx = (quat.y * uvz - quat.z * uvy) * 2.0;
        let uuvy = (quat.z * uvx - quat.x * uvz) * 2.0;
        let uuvz = (quat.x * uvy - quat.y * uvx) * 2.0;

        let w2 = quat.w * 2.0;
        uvx *= w2;
        uvy *= w2;
        uvz *= w2;

        self.x = self.x + uvx + uuvx;
        self.y = self.y + uvy + uuvy;
        self.z = self.z + uvz + uuvz;
    }
}

impl Add for Vec3 {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Vec3::new(self.x + other.x, self.y + other.y, self.z + other.z)
    }
}

impl AddAssign for Vec3 {
    fn add_assign(&mut self, other: Self) {
        self.x += other.x;
        self.y += other.y;
        self.z += other.z;
    }
}

impl Sub for Vec3 {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Vec3::new(self.x - other.x, self.y - other.y, self.z - other.z)
    }
}

impl SubAssign for Vec3 {
    fn sub_assign(&mut self, other: Self) {
        self.x -= other.x;
        self.y -= other.y;
        self.z -= other.z;
    }
}

impl Mul for Vec3 {
    type Output = Self;

    fn mul(self, other: Self) -> Self {
        Vec3::new(self.x * other.x, self.y * other.y, self.z * other.z)
    }
}

impl MulAssign for Vec3 {
    fn mul_assign(&mut self, other: Self) {
        self.x *= other.x;
        self.y *= other.y;
        self.z *= other.z;
    }
}

impl Mul<f32> for Vec3 {
    type Output = Self;

    fn mul(self, other: f32) -> Self {
        Vec3::new(self.x * other, self.y * other, self.z * other)
    }
}

impl MulAssign<f32> for Vec3 {
    fn mul_assign(&mut self, other: f32) {
        self.x *= other;
        self.y *= other;
        self.z *= other;
    }
}

impl Div for Vec3 {
    type Output = Self;

    fn div(self, other: Self) -> Self {
        Vec3::new(self.x / other.x, self.y / other.y, self.z / other.z)
    }
}

impl DivAssign for Vec3 {
    fn div_assign(&mut self, other: Self) {
        self.x /= other.x;
        self.y /= other.y;
        self.z /= other.z;
    }
}

impl Div<f32> for Vec3 {
    type Output = Vec3;

    fn div(self, other: f32) -> Self {
        Vec3::new(self.x / other, self.y / other, self.z / other)
    }
}

impl DivAssign<f32> for Vec3 {
    fn div_assign(&mut self, other: f32) {
        self.x /= other;
        self.y /= other;
        self.z /= other;
    }
}