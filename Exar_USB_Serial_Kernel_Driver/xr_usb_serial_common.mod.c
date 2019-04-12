#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x3e354fd8, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0xc2d0575b, __VMLINUX_SYMBOL_STR(usb_deregister) },
	{ 0x67b27ec1, __VMLINUX_SYMBOL_STR(tty_std_termios) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x692b6c9d, __VMLINUX_SYMBOL_STR(put_tty_driver) },
	{ 0x3cba09bc, __VMLINUX_SYMBOL_STR(tty_unregister_driver) },
	{ 0x7bd69bd4, __VMLINUX_SYMBOL_STR(usb_register_driver) },
	{ 0x125671cc, __VMLINUX_SYMBOL_STR(tty_register_driver) },
	{ 0x92a2c438, __VMLINUX_SYMBOL_STR(tty_set_operations) },
	{ 0xbddbeb10, __VMLINUX_SYMBOL_STR(__tty_alloc_driver) },
	{ 0x8a394388, __VMLINUX_SYMBOL_STR(tty_port_register_device) },
	{ 0x8c54f37c, __VMLINUX_SYMBOL_STR(usb_get_intf) },
	{ 0xb812890b, __VMLINUX_SYMBOL_STR(usb_driver_claim_interface) },
	{ 0x9aedd8e1, __VMLINUX_SYMBOL_STR(_dev_info) },
	{ 0xd2b09ce5, __VMLINUX_SYMBOL_STR(__kmalloc) },
	{ 0xb8026f6a, __VMLINUX_SYMBOL_STR(device_create_file) },
	{ 0xcce6b727, __VMLINUX_SYMBOL_STR(dev_warn) },
	{ 0x8c1db57d, __VMLINUX_SYMBOL_STR(usb_alloc_urb) },
	{ 0x12e753b1, __VMLINUX_SYMBOL_STR(usb_alloc_coherent) },
	{ 0x24632706, __VMLINUX_SYMBOL_STR(tty_port_init) },
	{ 0x1cd2b896, __VMLINUX_SYMBOL_STR(__mutex_init) },
	{ 0x11034fe7, __VMLINUX_SYMBOL_STR(usb_ifnum_to_if) },
	{ 0xd3acdbec, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0xceb0f5bd, __VMLINUX_SYMBOL_STR(cpu_hwcaps) },
	{ 0xc6cbbc89, __VMLINUX_SYMBOL_STR(capable) },
	{ 0x84bc974b, __VMLINUX_SYMBOL_STR(__arch_copy_from_user) },
	{ 0xb35dea8f, __VMLINUX_SYMBOL_STR(__arch_copy_to_user) },
	{ 0x1e79c33a, __VMLINUX_SYMBOL_STR(kmem_cache_alloc_trace) },
	{ 0x409873e3, __VMLINUX_SYMBOL_STR(tty_termios_baud_rate) },
	{ 0xef8188c5, __VMLINUX_SYMBOL_STR(tty_port_open) },
	{ 0x59745f1d, __VMLINUX_SYMBOL_STR(usb_autopm_put_interface) },
	{ 0xa7d39467, __VMLINUX_SYMBOL_STR(usb_autopm_get_interface) },
	{ 0x1a3c5860, __VMLINUX_SYMBOL_STR(tty_flip_buffer_push) },
	{ 0xaf0cb372, __VMLINUX_SYMBOL_STR(tty_insert_flip_string_fixed_flag) },
	{ 0x98c9d82f, __VMLINUX_SYMBOL_STR(usb_control_msg) },
	{ 0xd76af446, __VMLINUX_SYMBOL_STR(tty_standard_install) },
	{ 0xdbd1083c, __VMLINUX_SYMBOL_STR(__ll_sc___cmpxchg_case_mb_4) },
	{ 0xb35f25a0, __VMLINUX_SYMBOL_STR(tty_port_close) },
	{ 0x36c0866b, __VMLINUX_SYMBOL_STR(usb_autopm_get_interface_async) },
	{ 0x9eeae9b9, __VMLINUX_SYMBOL_STR(tty_port_hangup) },
	{ 0xcd7ef1f, __VMLINUX_SYMBOL_STR(tty_port_tty_wakeup) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0x43017438, __VMLINUX_SYMBOL_STR(usb_put_intf) },
	{ 0x2d3385d3, __VMLINUX_SYMBOL_STR(system_wq) },
	{ 0x2e0d2f7f, __VMLINUX_SYMBOL_STR(queue_work_on) },
	{ 0x91715312, __VMLINUX_SYMBOL_STR(sprintf) },
	{ 0xecfce6ba, __VMLINUX_SYMBOL_STR(tty_port_put) },
	{ 0xf1b4ed9d, __VMLINUX_SYMBOL_STR(usb_driver_release_interface) },
	{ 0xed94ef16, __VMLINUX_SYMBOL_STR(usb_free_urb) },
	{ 0x78991264, __VMLINUX_SYMBOL_STR(tty_unregister_device) },
	{ 0x11ec77f5, __VMLINUX_SYMBOL_STR(tty_kref_put) },
	{ 0xbdc82817, __VMLINUX_SYMBOL_STR(tty_vhangup) },
	{ 0xcc93a531, __VMLINUX_SYMBOL_STR(tty_port_tty_get) },
	{ 0x7d7b54a4, __VMLINUX_SYMBOL_STR(mutex_unlock) },
	{ 0x6f20465d, __VMLINUX_SYMBOL_STR(device_remove_file) },
	{ 0x3eff840c, __VMLINUX_SYMBOL_STR(mutex_lock) },
	{ 0x8c3971fa, __VMLINUX_SYMBOL_STR(usb_free_coherent) },
	{ 0xf33847d3, __VMLINUX_SYMBOL_STR(_raw_spin_unlock) },
	{ 0x5cd885d5, __VMLINUX_SYMBOL_STR(_raw_spin_lock) },
	{ 0x88bfa7e, __VMLINUX_SYMBOL_STR(cancel_work_sync) },
	{ 0x1b3e0a2e, __VMLINUX_SYMBOL_STR(usb_kill_urb) },
	{ 0xae8c4d0c, __VMLINUX_SYMBOL_STR(set_bit) },
	{ 0x9a908b80, __VMLINUX_SYMBOL_STR(test_and_clear_bit) },
	{ 0x6d5a0e60, __VMLINUX_SYMBOL_STR(usb_autopm_put_interface_async) },
	{ 0x526c3a6c, __VMLINUX_SYMBOL_STR(jiffies) },
	{ 0xa2972f75, __VMLINUX_SYMBOL_STR(__dynamic_dev_dbg) },
	{ 0x21840819, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0x44666304, __VMLINUX_SYMBOL_STR(usb_submit_urb) },
	{ 0x8a6f078a, __VMLINUX_SYMBOL_STR(tty_port_tty_hangup) },
	{ 0x4829a47e, __VMLINUX_SYMBOL_STR(memcpy) },
	{ 0x8fa8f791, __VMLINUX_SYMBOL_STR(_raw_spin_unlock_irq) },
	{ 0x20ffa7f6, __VMLINUX_SYMBOL_STR(_raw_spin_lock_irq) },
	{ 0x97fdbab9, __VMLINUX_SYMBOL_STR(_raw_spin_unlock_irqrestore) },
	{ 0x96220280, __VMLINUX_SYMBOL_STR(_raw_spin_lock_irqsave) },
	{ 0x1fdc7df2, __VMLINUX_SYMBOL_STR(_mcount) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("usb:v2890p0213d*dc*dsc*dp*ic*isc*ip*in*");
