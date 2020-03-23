use std::ops::{Add, AddAssign, Sub, SubAssign, Mul, MulAssign, Div, DivAssign};
use super::Vec4;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Mat4 {
    pub elements: [f32; 16]
}

impl Mat4 {
    pub fn identity() -> Self {
        let mut m = Self {
            elements: [0.0; 16]
        };

        m.elements[0] = 1.0;
        m.elements[5] = 1.0;
        m.elements[10] = 1.0;
        m.elements[15] = 1.0;

        m
    }

    
}

impl Mul<Vec4> for Mat4 {
    type Output = Vec4;

    fn mul(self, other: Vec4) -> Vec4 {
        let m = &self.elements;
        Vec4::new(
            m[0] * other.x() + m[4] * other.y() + m[8] * other.z() + m[12] * other.w(),
            m[1] * other.x() + m[5] * other.y() + m[9] * other.z() + m[13] * other.w(),
            m[2] * other.x() + m[6] * other.y() + m[10] * other.z() + m[14] * other.w(),
            m[3] * other.x() + m[7] * other.y() + m[11] * other.z() + m[15] * other.w()
        )
    }
}