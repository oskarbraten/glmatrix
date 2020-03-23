
mod vec3;
mod vec4;
mod quat;
mod mat4;

pub use vec3::Vec3;
pub use vec4::Vec4;
pub use quat::Quat;
pub use mat4::Mat4;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {

        assert_eq!(2 + 2, 4);
    }
}
