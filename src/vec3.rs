use num::Float;
use std::ops::{Add, AddAssign, Sub, SubAssign, Mul, MulAssign, Div, DivAssign, Neg};
use super::Quat;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Vec3<T: Float> {
    pub elements: [T; 3]
}

impl<T: Float> Vec3<T> {
    pub fn new(x: T, y: T, z: T) -> Self {
        Self {
            elements: [x, y, z]
        }
    }
    
    pub fn zero() -> Self {
        Self {
            elements: [T::zero(); 3]
        }
    }
    
    pub fn one() -> Self {
        Self {
            elements: [T::one(); 3]
        }
    }
    
    pub fn x(&self) -> T {
        self.elements[0]
    }
    
    pub fn y(&self) -> T {
        self.elements[1]
    }
    
    pub fn z(&self) -> T {
        self.elements[2]
    }
    
    pub fn set_x(&mut self, value: T) {
        self.elements[0] = value;
    }
    
    pub fn set_y(&mut self, value: T) {
        self.elements[1] = value;
    }
    
    pub fn set_z(&mut self, value: T) {
        self.elements[2] = value;
    }
    
    pub fn length_squared(&self) -> T {
        self.elements[0].powi(2) + self.elements[1].powi(2) + self.elements[2].powi(2)
    }
    
    pub fn length(&self) -> T {
        self.length_squared().sqrt()
    }
    
    pub fn magnitude(&self) -> T {
        self.length()
    }
    
    pub fn dist_squared(&self, other: &Self) -> T {
        let [ax, ay, az] = self.elements;
        let [bx, by, bz] = other.elements;

        (bx - ax).powi(2) + (by - ay).powi(2) + (bz - az).powi(2)
    }
    
    pub fn dist(&self, other: &Self) -> T {
        self.dist_squared(other).sqrt()
    }
    
    pub fn invert(&mut self) {
        self.elements[0] = T::one() / self.elements[0];
        self.elements[1] = T::one() / self.elements[1];
        self.elements[2] = T::one() / self.elements[2];
    }
    
    pub fn inverse(&self) -> Self {
        Self::new(
            T::one() / self.elements[0],
            T::one() / self.elements[1],
            T::one() / self.elements[2]
        )
    }
    
    pub fn normalize(&mut self) {
        let mut ls = self.length_squared();
        if ls > T::zero() {
            ls = T::one() / ls.sqrt();
        }
        
        self.elements[0] = self.elements[0] * ls;
        self.elements[1] = self.elements[1] * ls;
        self.elements[2] = self.elements[2] * ls;
    }
    
    pub fn normalized(&self) -> Self {
        let mut ls = self.length_squared();
        if ls > T::zero() {
            ls = T::one() / ls.sqrt();
        }
        
        Self::new(
            self.elements[0] * ls,
            self.elements[1] * ls,
            self.elements[2] * ls
        )
    }
    
    pub fn dot(&self, other: &Self) -> T {
        self.elements[0] * other.elements[0] + self.elements[1] * other.elements[1] + self.elements[2] * other.elements[2]
    }
    
    pub fn cross(&self, other: &Self) -> Self {
        let [ax, ay, az] = self.elements;
        let [bx, by, bz] = other.elements;
        
        Self::new(
            ay * bz - az * by,
            az * bx - ax * bz,
            ax * by - ay * bx
        )
    }
    
    pub fn lerp(&self, other: &Self, t: T) -> Self {
        let [ax, ay, az] = self.elements;
        let [bx, by, bz] = other.elements;

        Self::new(
            ax + t * (bx - ax),
            ay + t * (by - ay),
            az + t * (bz - az)
        )
    }
    
    pub fn rotate_quat(&mut self, quat: Quat<T>) {
        let [qx, qy, qz, qw] = quat.elements;
        
        let mut uvx = qy * self.elements[2] - qz * self.elements[1];
        let mut uvy = qz * self.elements[0] - qx * self.elements[2];
        let mut uvz = qx * self.elements[1] - qy * self.elements[0];
        
        let uuvx = (qy * uvz - qz * uvy) * T::from(2.0).unwrap();
        let uuvy = (qz * uvx - qx * uvz) * T::from(2.0).unwrap();
        let uuvz = (qx * uvy - qy * uvx) * T::from(2.0).unwrap();
        
        let w2 = qw * T::from(2.0).unwrap();
        uvx = uvx * w2;
        uvy = uvy * w2;
        uvz = uvz * w2;
        
        self.elements[0] = self.elements[0] + uvx + uuvx;
        self.elements[1] = self.elements[1] + uvy + uuvy;
        self.elements[2] = self.elements[2] + uvz + uuvz;
    }
}

impl<T: Float> Add for Vec3<T> {
    type Output = Self;
    
    fn add(self, other: Self) -> Self {
        Self::new(
            self.elements[0] + other.elements[0],
            self.elements[1] + other.elements[1],
            self.elements[2] + other.elements[2]
        )
    }
}

impl<T: Float + AddAssign> AddAssign for Vec3<T> {
    fn add_assign(&mut self, other: Self) {
        self.elements[0] += other.elements[0];
        self.elements[1] += other.elements[1];
        self.elements[2] += other.elements[2];
    }
}

impl<T: Float> Sub for Vec3<T> {
    type Output = Self;
    
    fn sub(self, other: Self) -> Self {
        Self::new(
            self.elements[0] - other.elements[0],
            self.elements[1] - other.elements[1],
            self.elements[2] - other.elements[2]
        )
    }
}

impl<T: Float + SubAssign> SubAssign for Vec3<T> {
    fn sub_assign(&mut self, other: Self) {
        self.elements[0] -= other.elements[0];
        self.elements[1] -= other.elements[1];
        self.elements[2] -= other.elements[2];
    }
}

impl<T: Float> Neg for Vec3<T> {
    type Output = Self;
    
    fn neg(self) -> Self {
        Self::new(-self.elements[0], -self.elements[1], -self.elements[2])
    }
}

impl<T: Float> Mul for Vec3<T> {
    type Output = Self;
    
    fn mul(self, other: Self) -> Self {
        Self::new(
            self.elements[0] * other.elements[0],
            self.elements[1] * other.elements[1],
            self.elements[2] * other.elements[2]
        )
    }
}

impl<T: Float + MulAssign> MulAssign for Vec3<T> {
    fn mul_assign(&mut self, other: Self) {
        self.elements[0] *= other.elements[0];
        self.elements[1] *= other.elements[1];
        self.elements[2] *= other.elements[2];
    }
}

impl<T: Float> Mul<T> for Vec3<T> {
    type Output = Self;
    
    fn mul(self, other: T) -> Self {
        Self::new(
            self.elements[0] * other,
            self.elements[1] * other,
            self.elements[2] * other
        )
    }
}

impl<T: Float + MulAssign> MulAssign<T> for Vec3<T> {
    fn mul_assign(&mut self, other: T) {
        self.elements[0] *= other;
        self.elements[1] *= other;
        self.elements[2] *= other;
    }
}

impl<T: Float> Div for Vec3<T> {
    type Output = Self;
    
    fn div(self, other: Self) -> Self {
        Self::new(
            self.elements[0] / other.elements[0],
            self.elements[1] / other.elements[1],
            self.elements[2] / other.elements[2]
        )
    }
}

impl<T: Float + DivAssign> DivAssign for Vec3<T> {
    fn div_assign(&mut self, other: Self) {
        self.elements[0] /= other.elements[0];
        self.elements[1] /= other.elements[1];
        self.elements[2] /= other.elements[2];
    }
}

impl<T: Float> Div<T> for Vec3<T> {
    type Output = Vec3<T>;
    
    fn div(self, other: T) -> Self {
        Self::new(
            self.elements[0] / other,
            self.elements[1] / other,
            self.elements[2] / other
        )
    }
}

impl<T: Float + DivAssign> DivAssign<T> for Vec3<T> {
    fn div_assign(&mut self, other: T) {
        self.elements[0] /= other;
        self.elements[1] /= other;
        self.elements[2] /= other;
    }
}