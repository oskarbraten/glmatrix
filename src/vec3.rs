use std::ops::{Add, AddAssign, Sub, SubAssign, Mul, MulAssign, Div, DivAssign, Neg};
use super::Quat;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Vec3 {
    pub elements: [f32; 3]
}

impl Vec3 {
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self {
            elements: [x, y, z]
        }
    }
    
    pub const fn zero() -> Self {
        Self {
            elements: [0.0; 3]
        }
    }
    
    pub const fn one() -> Self {
        Self {
            elements: [1.0; 3]
        }
    }
    
    pub fn x(&self) -> f32 {
        self.elements[0]
    }
    
    pub fn y(&self) -> f32 {
        self.elements[1]
    }
    
    pub fn z(&self) -> f32 {
        self.elements[2]
    }
    
    pub fn set_x(&mut self, value: f32) {
        self.elements[0] = value;
    }
    
    pub fn set_y(&mut self, value: f32) {
        self.elements[1] = value;
    }
    
    pub fn set_z(&mut self, value: f32) {
        self.elements[2] = value;
    }
    
    pub fn length_squared(&self) -> f32 {
        self.elements[0].powi(2) + self.elements[1].powi(2) + self.elements[2].powi(2)
    }
    
    pub fn length(&self) -> f32 {
        self.length_squared().sqrt()
    }
    
    pub fn magnitude(&self) -> f32 {
        self.length()
    }
    
    pub fn dist_squared(&self, other: &Self) -> f32 {
        let [ax, ay, az] = &self.elements;
        let [bx, by, bz] = &other.elements;

        (bx - ax).powi(2) + (by - ay).powi(2) + (bz - az).powi(2)
    }
    
    pub fn dist(&self, other: &Self) -> f32 {
        self.dist_squared(other).sqrt()
    }
    
    pub fn invert(&mut self) {
        self.elements[0] = 1.0 / self.elements[0];
        self.elements[1] = 1.0 / self.elements[1];
        self.elements[2] = 1.0 / self.elements[2];
    }
    
    pub fn inverse(&self) -> Self {
        Self::new(
            1.0 / self.elements[0],
            1.0 / self.elements[1],
            1.0 / self.elements[2]
        )
    }
    
    pub fn normalize(&mut self) {
        let mut ls = self.length_squared();
        if ls > 0.0 {
            ls = 1.0 / ls.sqrt();
        }
        
        self.elements[0] *= ls;
        self.elements[1] *= ls;
        self.elements[2] *= ls;
    }
    
    pub fn normalized(&self) -> Self {
        let mut ls = self.length_squared();
        if ls > 0.0 {
            ls = 1.0 / ls.sqrt();
        }
        
        Self::new(
            self.elements[0] * ls,
            self.elements[1] * ls,
            self.elements[2] * ls
        )
    }
    
    pub fn dot(&self, other: &Self) -> f32 {
        self.elements[0] * other.elements[0] + self.elements[1] * other.elements[1] + self.elements[2] * other.elements[2]
    }
    
    pub fn cross(&self, other: &Self) -> Self {
        let [ax, ay, az] = &self.elements;
        let [bx, by, bz] = &other.elements;
        
        Self::new(
            ay * bz - az * by,
            az * bx - ax * bz,
            ax * by - ay * bx
        )
    }
    
    pub fn lerp(&self, other: &Self, t: f32) -> Self {
        let [ax, ay, az] = &self.elements;
        let [bx, by, bz] = &other.elements;

        Self::new(
            ax + t * (bx - ax),
            ay + t * (by - ay),
            az + t * (bz - az)
        )
    }
    
    pub fn rotate_quat(&mut self, quat: Quat) {
        let [qx, qy, qz, qw] = &quat.elements;
        
        let mut uvx = qy * self.elements[2] - qz * self.elements[1];
        let mut uvy = qz * self.elements[0] - qx * self.elements[2];
        let mut uvz = qx * self.elements[1] - qy * self.elements[0];
        
        let uuvx = (qy * uvz - qz * uvy) * 2.0;
        let uuvy = (qz * uvx - qx * uvz) * 2.0;
        let uuvz = (qx * uvy - qy * uvx) * 2.0;
        
        let w2 = qw * 2.0;
        uvx *= w2;
        uvy *= w2;
        uvz *= w2;
        
        self.elements[0] = self.elements[0] + uvx + uuvx;
        self.elements[1] = self.elements[1] + uvy + uuvy;
        self.elements[2] = self.elements[2] + uvz + uuvz;
    }
}

impl Add for Vec3 {
    type Output = Self;
    
    fn add(self, other: Self) -> Self {
        Self::new(
            self.elements[0] + other.elements[0],
            self.elements[1] + other.elements[1],
            self.elements[2] + other.elements[2]
        )
    }
}

impl AddAssign for Vec3 {
    fn add_assign(&mut self, other: Self) {
        self.elements[0] += other.elements[0];
        self.elements[1] += other.elements[1];
        self.elements[2] += other.elements[2];
    }
}

impl Sub for Vec3 {
    type Output = Self;
    
    fn sub(self, other: Self) -> Self {
        Self::new(
            self.elements[0] - other.elements[0],
            self.elements[1] - other.elements[1],
            self.elements[2] - other.elements[2]
        )
    }
}

impl SubAssign for Vec3 {
    fn sub_assign(&mut self, other: Self) {
        self.elements[0] -= other.elements[0];
        self.elements[1] -= other.elements[1];
        self.elements[2] -= other.elements[2];
    }
}

impl Neg for Vec3 {
    type Output = Self;
    
    fn neg(self) -> Self {
        Self::new(-self.elements[0], -self.elements[1], -self.elements[2])
    }
}

impl Mul for Vec3 {
    type Output = Self;
    
    fn mul(self, other: Self) -> Self {
        Self::new(
            self.elements[0] * other.elements[0],
            self.elements[1] * other.elements[1],
            self.elements[2] * other.elements[2]
        )
    }
}

impl MulAssign for Vec3 {
    fn mul_assign(&mut self, other: Self) {
        self.elements[0] *= other.elements[0];
        self.elements[1] *= other.elements[1];
        self.elements[2] *= other.elements[2];
    }
}

impl Mul<f32> for Vec3 {
    type Output = Self;
    
    fn mul(self, other: f32) -> Self {
        Self::new(
            self.elements[0] * other,
            self.elements[1] * other,
            self.elements[2] * other
        )
    }
}

impl MulAssign<f32> for Vec3 {
    fn mul_assign(&mut self, other: f32) {
        self.elements[0] *= other;
        self.elements[1] *= other;
        self.elements[2] *= other;
    }
}

impl Div for Vec3 {
    type Output = Self;
    
    fn div(self, other: Self) -> Self {
        Self::new(
            self.elements[0] / other.elements[0],
            self.elements[1] / other.elements[1],
            self.elements[2] / other.elements[2]
        )
    }
}

impl DivAssign for Vec3 {
    fn div_assign(&mut self, other: Self) {
        self.elements[0] /= other.elements[0];
        self.elements[1] /= other.elements[1];
        self.elements[2] /= other.elements[2];
    }
}

impl Div<f32> for Vec3 {
    type Output = Vec3;
    
    fn div(self, other: f32) -> Self {
        Self::new(
            self.elements[0] / other,
            self.elements[1] / other,
            self.elements[2] / other
        )
    }
}

impl DivAssign<f32> for Vec3 {
    fn div_assign(&mut self, other: f32) {
        self.elements[0] /= other;
        self.elements[1] /= other;
        self.elements[2] /= other;
    }
}