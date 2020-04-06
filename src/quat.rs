use num::Float;
use std::ops::{Mul, MulAssign};
use super::{Vec3, Vec4};

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Quat<T> {
    pub elements: [T; 4]
}

impl<T> Quat<T> {
    pub const fn new(x: T, y: T, z: T, w: T) -> Self {
        Self {
            elements: [x, y, z, w]
        }
    }
}

impl Quat<f32> {
    pub fn as_bytes(&self) -> [u8; 16] {
        let mut bytes = [0u8; 16];

        for (i, v) in self.elements.iter().enumerate() {
            let offset = i * std::mem::size_of::<f32>();

            let b = v.to_ne_bytes();

            bytes[offset + 0] = b[0];
            bytes[offset + 1] = b[1];
            bytes[offset + 2] = b[2];
            bytes[offset + 3] = b[3];
        }

        bytes
    }

    /// Creates a Quat<f32> from a slice of 16 bytes.
    pub fn from_bytes(bytes: &[u8; 16]) -> Self {
        let mut elements = [0.0f32; 4];
        for i in 0..4 {

            let offset = i * 4;
            elements[i] = f32::from_ne_bytes([bytes[offset + 0], bytes[offset + 1], bytes[offset + 2], bytes[offset + 3]]);

        }

        Self {
            elements
        }
    }
}

impl Quat<f64> {
    pub fn as_bytes(&self) -> [u8; 32] {
        let mut bytes = [0u8; 32];

        for (i, v) in self.elements.iter().enumerate() {
            let offset = i * std::mem::size_of::<f64>();
            let b = v.to_ne_bytes();

            bytes[offset + 0] = b[0];
            bytes[offset + 1] = b[1];
            bytes[offset + 2] = b[2];
            bytes[offset + 3] = b[3];
            bytes[offset + 4] = b[4];
            bytes[offset + 5] = b[5];
            bytes[offset + 6] = b[6];
            bytes[offset + 7] = b[7];
        }

        bytes
    }

    /// Creates a Quat<f64> from a slice of 32 bytes.
    pub fn from_bytes(bytes: &[u8; 32]) -> Self {
        let mut elements = [0.0f64; 4];
        for i in 0..4 {

            let offset = i * 8;
            elements[i] = f64::from_ne_bytes([
                bytes[offset + 0],
                bytes[offset + 1],
                bytes[offset + 2],
                bytes[offset + 3],
                bytes[offset + 4],
                bytes[offset + 5],
                bytes[offset + 6],
                bytes[offset + 7]
            ]);

        }

        Self {
            elements
        }
    }
}

impl<T: Float + MulAssign> Quat<T> {
    pub fn identity() -> Self {
        Self::new(T::zero(), T::zero(), T::zero(), T::one())
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

    pub fn w(&self) -> T {
        self.elements[3]
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

    pub fn set_w(&mut self, value: T) {
        self.elements[3] = value;
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

impl<T: Float + MulAssign> From<[T; 4]> for Quat<T> {
    fn from(v: [T; 4]) -> Self {
        Self::new(v[0], v[1], v[2], v[3])
    }
}

impl From<Quat<f32>> for Quat<f64> {
    fn from(v: Quat<f32>) -> Self {
        Self::new(v.x() as f64, v.y() as f64, v.z() as f64, v.w() as f64)
    }
}

impl From<Quat<f64>> for Quat<f32> {
    fn from(v: Quat<f64>) -> Self {
        Self::new(v.x() as f32, v.y() as f32, v.z() as f32, v.w() as f32)
    }
}

impl<T: Float, Index> std::ops::Index<Index> for Quat<T>
where
    Index: std::slice::SliceIndex<[T]>
{
    type Output = Index::Output;

    fn index(&self, index: Index) -> &Self::Output {
        &self.elements[index]
    }
}

impl<T: Float, Index> std::ops::IndexMut<Index> for Quat<T>
where
    Index: std::slice::SliceIndex<[T]>
{
    fn index_mut(&mut self, index: Index) -> &mut Self::Output {
        &mut self.elements[index]
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