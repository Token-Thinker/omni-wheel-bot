//! Shared board identity types used to describe targets without tying the
//! project to one HAL or MCU family.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoardFamily {
    Esp32S3,
    HostSim,
    Rp2040,
    Stm32,
    ArduinoClass,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoardRuntimeKind {
    CameraOnly,
    DevKitReference,
    OwbS3V1,
    OwbS3V2,
    HostSimulation,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CpuTopology {
    SingleCore,
    DualCore,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RuntimeExecutionPlan {
    pub topology: CpuTopology,
    /// Logical CPU index for control/PID command execution.
    pub control_cpu: u8,
    /// Logical CPU index for network/WebSocket services.
    pub network_cpu: u8,
    /// Logical CPU index for camera/media services.
    pub media_cpu: u8,
}

impl RuntimeExecutionPlan {
    pub const fn single_core(cpu: u8) -> Self {
        Self {
            topology: CpuTopology::SingleCore,
            control_cpu: cpu,
            network_cpu: cpu,
            media_cpu: cpu,
        }
    }

    pub const fn dual_core(
        control_cpu: u8,
        network_cpu: u8,
        media_cpu: u8,
    ) -> Self {
        Self {
            topology: CpuTopology::DualCore,
            control_cpu,
            network_cpu,
            media_cpu,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{CpuTopology, RuntimeExecutionPlan};

    #[test]
    fn single_core_plan_maps_all_lanes_to_same_cpu() {
        let plan = RuntimeExecutionPlan::single_core(0);
        assert_eq!(plan.topology, CpuTopology::SingleCore);
        assert_eq!(plan.control_cpu, 0);
        assert_eq!(plan.network_cpu, 0);
        assert_eq!(plan.media_cpu, 0);
    }

    #[test]
    fn dual_core_plan_keeps_explicit_lane_mapping() {
        let plan = RuntimeExecutionPlan::dual_core(1, 0, 0);
        assert_eq!(plan.topology, CpuTopology::DualCore);
        assert_eq!(plan.control_cpu, 1);
        assert_eq!(plan.network_cpu, 0);
        assert_eq!(plan.media_cpu, 0);
    }
}
