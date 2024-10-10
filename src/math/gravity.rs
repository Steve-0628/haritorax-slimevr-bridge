use core::fmt::Debug;
use super::Rotation;

pub struct Gravity {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Gravity {
    pub fn to_vector3(&self, rotation: &Rotation) -> Gravity {
        let rotation_components = [
            rotation.w,
            rotation.x,
            rotation.y,
            rotation.z,
        ];
        let rotation = [
            rotation.w,
            -rotation.x,
            -rotation.y,
            -rotation.z,
        ];
        let gravity = [0f32, 0f32, 0f32, 9.81f32];
        let hrp = [
            rotation[0] * gravity[0] - rotation[1] * gravity[1] - rotation[2] * gravity[2] - rotation[3] * gravity[3],
            rotation[0] * gravity[1] + rotation[1] * gravity[0] + rotation[2] * gravity[3] - rotation[3] * gravity[2],
            rotation[0] * gravity[2] - rotation[1] * gravity[3] + rotation[2] * gravity[0] + rotation[3] * gravity[1],
            rotation[0] * gravity[3] + rotation[1] * gravity[2] - rotation[2] * gravity[1] + rotation[3] * gravity[0],
        ]; 
        let h_final = [
            // hrp[0] * rotation_components[0] - hrp[1] * rotation_components[1] - hrp[2] * rotation_components[2] - hrp[3] * rotation_components[3],
            hrp[0] * rotation_components[1] + hrp[1] * rotation_components[0] + hrp[2] * rotation_components[3] - hrp[3] * rotation_components[2],
            hrp[0] * rotation_components[2] - hrp[1] * rotation_components[3] + hrp[2] * rotation_components[0] + hrp[3] * rotation_components[1],
            hrp[0] * rotation_components[3] + hrp[1] * rotation_components[2] - hrp[2] * rotation_components[1] + hrp[3] * rotation_components[0],
        ];
        Gravity {
            x: self.x / 256f32 - h_final[0] * -1.2,
            y: self.y / 256f32 - h_final[1] * -1.2,
            z: self.z / 256f32 - h_final[2] * 1.2,
        }
    }
}

impl Debug for Gravity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Gravity: ({:06.2}, {:06.2}, {:06.2})", self.x, self.y, self.z)
    }
}
