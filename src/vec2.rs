use num::{Num, Float, Signed};
use std::ops::{Add, AddAssign, Sub, SubAssign, Mul, MulAssign, Div, DivAssign, Neg};

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Vec2<T> {
    pub elements: [T; 2]
}

impl<T: Num + Copy> Vec2<T> {
    pub fn new(x: T, y: T) -> Self {
        Self {
            elements: [x, y]
        }
    }
    
    pub fn zero() -> Self {
        Self {
            elements: [T::zero(); 2]
        }
    }
    
    pub fn one() -> Self {
        Self {
            elements: [T::one(); 2]
        }
    }
    
    pub fn x(&self) -> T {
        self.elements[0]
    }
    
    pub fn y(&self) -> T {
        self.elements[1]
    }
    
    pub fn set_x(&mut self, value: T) {
        self.elements[0] = value;
    }
    
    pub fn set_y(&mut self, value: T) {
        self.elements[1] = value;
    }
    
    pub fn invert(&mut self) {
        self.elements[0] = T::one() / self.elements[0];
        self.elements[1] = T::one() / self.elements[1];
    }
    
    pub fn inverse(&self) -> Self {
        Self::new(
            T::one() / self.elements[0],
            T::one() / self.elements[1]
        )
    }
    
    pub fn dot(&self, other: &Self) -> T {
        self.elements[0] * other.elements[0] + self.elements[1] * other.elements[1]
    }
}

impl<T: Num + Copy + Float> Vec2<T> {
    pub fn length_squared(&self) -> T {
        self.elements[0].powi(2) + self.elements[1].powi(2)
    }
    
    pub fn length(&self) -> T {
        self.length_squared().sqrt()
    }
    
    pub fn magnitude(&self) -> T {
        self.length()
    }
    
    pub fn dist_squared(&self, other: &Self) -> T {
        let [ax, ay] = self.elements;
        let [bx, by] = other.elements;

        (bx - ax).powi(2) + (by - ay).powi(2)
    }
    
    pub fn dist(&self, other: &Self) -> T {
        self.dist_squared(other).sqrt()
    }

    pub fn normalize(&mut self) {
        let mut ls = self.length_squared();
        if ls > T::zero() {
            ls = T::one() / ls.sqrt();
        }
        
        self.elements[0] = self.elements[0] * ls;
        self.elements[1] = self.elements[1] * ls;
    }
    
    pub fn normalized(&self) -> Self {
        let mut ls = self.length_squared();
        if ls > T::zero() {
            ls = T::one() / ls.sqrt();
        }
        
        Self::new(
            self.elements[0] * ls,
            self.elements[1] * ls
        )
    }

    pub fn lerp(&self, other: &Self, t: T) -> Self {
        let [ax, ay] = self.elements;
        let [bx, by] = other.elements;

        Self::new(
            ax + t * (bx - ax),
            ay + t * (by - ay)
        )
    }

    pub fn rotate(&mut self, origin: &Self, rad: T) {
        let p0 = self.elements[0] - origin.elements[0];
        let p1 = self.elements[1] - origin.elements[1];

        let s = rad.sin();
        let c = rad.cos();

        self.elements[0] = p0 * c - p1 * s + origin.elements[0];
        self.elements[1] = p0 * s - p1 * c + origin.elements[1];
    }
    
    pub fn rotated(&self, origin: &Self, rad: T) -> Self {
        let p0 = self.elements[0] - origin.elements[0];
        let p1 = self.elements[1] - origin.elements[1];

        let s = rad.sin();
        let c = rad.cos();

        Vec2::new(p0 * c - p1 * s + origin.elements[0], p0 * s - p1 * c + origin.elements[1])
    }

    pub fn angle(&self, other: &Self) -> T {
        let [x1, y1] = self.elements;
        let [x2, y2] = other.elements;

        let m = (x1 * x1 + y1 * y1).sqrt() * (x2 * x2 + y2 * y2).sqrt();
        
        let cos = if m == T::zero() {
            T::zero()
        } else {
            (x1 * x2 + y1 * y2) / m
        };

        num::clamp(cos, -T::one(), T::one()).acos()
    }
}

impl From<Vec2<f32>> for Vec2<f64> {
    fn from(v: Vec2<f32>) -> Self {
        Self::new(v.x() as f64, v.y() as f64)
    }
}

impl From<Vec2<f64>> for Vec2<f32> {
    fn from(v: Vec2<f64>) -> Self {
        Self::new(v.x() as f32, v.y() as f32)
    }
}

impl<T, Index> std::ops::Index<Index> for Vec2<T>
where
    Index: std::slice::SliceIndex<[T]>
{
    type Output = Index::Output;

    fn index(&self, index: Index) -> &Self::Output {
        &self.elements[index]
    }
}

impl<T: Num + Copy> Add for Vec2<T> {
    type Output = Self;
    
    fn add(self, other: Self) -> Self {
        Self::new(
            self.elements[0] + other.elements[0],
            self.elements[1] + other.elements[1]
        )
    }
}

impl<T: Num + Copy + AddAssign> AddAssign for Vec2<T> {
    fn add_assign(&mut self, other: Self) {
        self.elements[0] += other.elements[0];
        self.elements[1] += other.elements[1];
    }
}

impl<T: Num + Copy> Sub for Vec2<T> {
    type Output = Self;
    
    fn sub(self, other: Self) -> Self {
        Self::new(
            self.elements[0] - other.elements[0],
            self.elements[1] - other.elements[1]
        )
    }
}

impl<T: Num + Copy + SubAssign> SubAssign for Vec2<T> {
    fn sub_assign(&mut self, other: Self) {
        self.elements[0] -= other.elements[0];
        self.elements[1] -= other.elements[1];
    }
}

impl<T: Num + Copy + Signed> Neg for Vec2<T> {
    type Output = Self;
    
    fn neg(self) -> Self {
        Self::new(-self.elements[0], -self.elements[1])
    }
}

impl<T: Num + Copy> Mul for Vec2<T> {
    type Output = Self;
    
    fn mul(self, other: Self) -> Self {
        Self::new(
            self.elements[0] * other.elements[0],
            self.elements[1] * other.elements[1]
        )
    }
}

impl<T: Num + Copy + MulAssign> MulAssign for Vec2<T> {
    fn mul_assign(&mut self, other: Self) {
        self.elements[0] *= other.elements[0];
        self.elements[1] *= other.elements[1];
    }
}

impl<T: Num + Copy> Mul<T> for Vec2<T> {
    type Output = Self;
    
    fn mul(self, other: T) -> Self {
        Self::new(
            self.elements[0] * other,
            self.elements[1] * other
        )
    }
}

impl<T: Num + Copy + MulAssign> MulAssign<T> for Vec2<T> {
    fn mul_assign(&mut self, other: T) {
        self.elements[0] *= other;
        self.elements[1] *= other;
    }
}

impl<T: Num + Copy> Div for Vec2<T> {
    type Output = Self;
    
    fn div(self, other: Self) -> Self {
        Self::new(
            self.elements[0] / other.elements[0],
            self.elements[1] / other.elements[1]
        )
    }
}

impl<T: Num + Copy + DivAssign> DivAssign for Vec2<T> {
    fn div_assign(&mut self, other: Self) {
        self.elements[0] /= other.elements[0];
        self.elements[1] /= other.elements[1];
    }
}

impl<T: Num + Copy> Div<T> for Vec2<T> {
    type Output = Self;
    
    fn div(self, other: T) -> Self {
        Self::new(
            self.elements[0] / other,
            self.elements[1] / other
        )
    }
}

impl<T: Num + Copy + DivAssign> DivAssign<T> for Vec2<T> {
    fn div_assign(&mut self, other: T) {
        self.elements[0] /= other;
        self.elements[1] /= other;
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new() {
        assert_eq!(Vec2::new(0.0f32, 0.0), Vec2 { elements: [0.0f32; 2] });
        assert_eq!(Vec2::new(0.0f64, 0.0), Vec2 { elements: [0.0f64; 2] });

        assert_eq!(Vec2::new(0u8, 0u8), Vec2 { elements: [0u8; 2] });
        assert_eq!(Vec2::new(0u16, 0u16), Vec2 { elements: [0u16; 2] });
        assert_eq!(Vec2::new(0u32, 0u32), Vec2 { elements: [0u32; 2] });
        assert_eq!(Vec2::new(0u64, 0u64), Vec2 { elements: [0u64; 2] });
        assert_eq!(Vec2::new(0u128, 0u128), Vec2 { elements: [0u128; 2] });

        assert_eq!(Vec2::new(0i8, 0i8), Vec2 { elements: [0i8; 2] });
        assert_eq!(Vec2::new(0i16, 0i16), Vec2 { elements: [0i16; 2] });
        assert_eq!(Vec2::new(0i32, 0i32), Vec2 { elements: [0i32; 2] });
        assert_eq!(Vec2::new(0i64, 0i64), Vec2 { elements: [0i64; 2] });
        assert_eq!(Vec2::new(0i128, 0i128), Vec2 { elements: [0i128; 2] });
    }

    #[test]
    fn zero() {
        assert_eq!(Vec2::zero(), Vec2 { elements: [0.0f32; 2] });
        assert_eq!(Vec2::zero(), Vec2 { elements: [0.0f64; 2] });
    }

    #[test]
    fn one() {
        assert_eq!(Vec2::one(), Vec2 { elements: [1.0f32; 2] });
        assert_eq!(Vec2::one(), Vec2 { elements: [1.0f64; 2] });
    }

    #[test]
    fn getters() {
        let v32 = Vec2::new(0.0f32, 1.0);
        let v64 = Vec2::new(0.0f64, 1.0);

        assert_eq!(v32.x(), 0.0);
        assert_eq!(v32.y(), 1.0);

        assert_eq!(v64.x(), 0.0);
        assert_eq!(v64.y(), 1.0);
    }

    #[test]
    fn setters() {
        let mut v = Vec2::new(0.0f32, 1.0);

        assert_eq!(v.x(), 0.0);
        v.set_x(1.0);
        assert_eq!(v.x(), 1.0);

        assert_eq!(v.y(), 1.0);
        v.set_y(2.0);
        assert_eq!(v.y(), 2.0);
    }

    #[test]
    fn length_squared() {
        let v = Vec2::new(4.0, 0.0);
        assert_eq!(v.length_squared(), 16.0);
    }

    #[test]
    fn length() {
        let v = Vec2::new(4.0, 0.0);
        assert_eq!(v.length(), (4.0 * 4.0).sqrt());
    }

    #[test]
    fn dist_squared() {
        let v1 = Vec2::new(0.0, 0.0);
        let v2 = Vec2::new(4.0, 0.0);
        assert_eq!(v1.dist_squared(&v2), 16.0);
    }

    #[test]
    fn dist() {
        let v1 = Vec2::new(0.0, 0.0);
        let v2 = Vec2::new(4.0, 0.0);
        assert_eq!(v1.dist(&v2), 4.0);
    }

    #[test]
    fn invert() {
        let mut v = Vec2::new(1.0, 2.0);
        v.invert();
        assert_eq!(v, Vec2::new(1.0 / 1.0, 1.0 / 2.0));
    }

    #[test]
    fn inverse() {
        let v = Vec2::new(1.0, 2.0);
        assert_eq!(v.inverse(), Vec2::new(1.0 / 1.0, 1.0 / 2.0));
    }

    #[test]
    fn normalize() {
        let mut v = Vec2::new(2.0, 0.0);
        v.normalize();

        assert_eq!(v, Vec2::new(1.0, 0.0));
    }

    #[test]
    fn normalized() {
        let v = Vec2::new(2.0, 0.0);
        assert_eq!(v.normalized(), Vec2::new(1.0, 0.0));
    }

    #[test]
    fn dot() {
        let v1 = Vec2::new(2.0, 2.0);
        let v2 = Vec2::new(2.0, 2.0);
        assert_eq!(v1.dot(&v2), 4.0 + 4.0);
    }

    // TODO: add check with approximation.
    // #[test]
    // fn rotate() {
    //     let mut v1 = Vec2::new(1.0, 0.0);
    //     let v2 = Vec2::new(0.0, 1.0);

    //     v1.rotate(&Vec2::zero(), std::f32::consts::PI / 2.0);
    //     assert_eq!(v1, v2);
    // }

    #[test]
    fn angle() {
        let v1 = Vec2::new(1.0, 0.0);
        let v2 = Vec2::new(0.0, 1.0);

        assert_eq!(v1.angle(&v2), std::f32::consts::PI / 2.0);
    }
}