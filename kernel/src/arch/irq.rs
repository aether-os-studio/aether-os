pub trait IrqRegsArch {
    fn get_ip(&self) -> u64;
    fn set_ip(&mut self, ip: u64);
    fn get_sp(&self) -> u64;
    fn set_sp(&mut self, sp: u64);

    fn get_ret_value(&self) -> u64;
    fn set_ret_value(&mut self, ret_value: u64);
    fn get_ret_address(&self) -> u64;
    fn set_ret_address(&mut self, ret_address: u64);

    fn get_args(&self) -> (u64, u64, u64, u64, u64, u64);
    fn set_args(&mut self, args: (u64, u64, u64, u64, u64, u64));

    fn get_syscall_idx(&self) -> u64;
    fn get_syscall_args(&self) -> (u64, u64, u64, u64, u64, u64);

    fn set_user_space(&mut self, user: bool);
}

pub trait IrqArch {
    fn enable_global_irq();
    fn disable_global_irq();
}

pub trait IrqControllerArch {
    fn enable_irq(&mut self, irq: usize);
    fn disable_irq(&mut self, irq: usize);
    fn send_eoi(&mut self, irq: usize);
}
