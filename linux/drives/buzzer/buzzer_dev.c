/****************************************************************
  Copyright (C) 2016 Sean Guo. All rights reserved.

  > File Name:         < buzzer_dev.c >
  > Author:            < Sean Guo >
  > Mail:              < iseanxp+code@gmail.com >
  > Created Time:      < 2016/01/18 >
  > Description:      FriendlyARM - Tiny6410 - Linux Device Drivers - Buttons - 混杂设备miscdevice驱动
  TinyADK-1312_sch.pdf: Buzzer1  - XpwmTOUT0/XCLKOUT/GPF14
 ****************************************************************/
//{{{ include
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <mach/hardware.h>
#include <plat/regs-timer.h>
#include <mach/regs-irq.h>
#include <asm/mach/time.h>
#include <linux/clk.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>

#include <plat/gpio-cfg.h>
#include <mach/gpio-bank-e.h>
#include <mach/gpio-bank-f.h>
#include <mach/gpio-bank-k.h>
//}}}
// {{{ Device Name
// 加载模式后，执行"cat /proc/devices"命令看到的设备名称
#define DEVICE_NAME     "buzzerdev"
// 设备编号, 为0时会自动分配
#define DEVICE_MAJOR    0
//}}}
//{{{ define
#define PWM_IOCTL_SET_FREQ      1  //定义宏变量，用于后面的ioctl中的switch case
#define PWM_IOCTL_STOP          0  //定义信号量lock
//}}}

static struct semaphore lock;

//{{{ PWM_set_freq
/* freq:  pclk/50/16/65536 ~ pclk/50/16
 * if pclk = 50MHz, freq is 1Hz to 62500Hz
 * human ear : 20Hz~ 20000Hz
 */
static void PWM_Set_Freq( unsigned long freq )  //设置pwm的频率，配置各个寄存器
{
    unsigned long tcon;
    unsigned long tcnt;
    unsigned long tcfg1;
    unsigned long tcfg0;

    struct clk *clk_p;
    unsigned long pclk;

    unsigned tmp;

    tmp = readl(S3C64XX_GPFCON);  //参考第一个表，设置GPF14为TOUT0，pwm输出
    tmp &= ~(0x3U << 28);
    tmp |=  (0x2U << 28);
    writel(tmp, S3C64XX_GPFCON);

    tcon = __raw_readl(S3C_TCON);    //读定时器配置寄存器TCON到tcon
    tcfg1 = __raw_readl(S3C_TCFG1);  //读寄存器TCFG1到tcfg1
    tcfg0 = __raw_readl(S3C_TCFG0);  //读寄存器TCFG0到tcfg0

    //prescaler = 50
    tcfg0 &= ~S3C_TCFG_PRESCALER0_MASK;  S3C_TCFG_PRESCALER0_MASK=255(11111111),为定时器0和1的预分频值得掩码，TCFG[0~8]
        tcfg0 |= (50 - 1);   tcfg0=00110001,预分频为50

        //mux = 1/16
        tcfg1 &= ~S3C_TCFG1_MUX0_MASK;  //定时器0分割值得掩码 (15<<0)
    tcfg1 |= S3C_TCFG1_MUX0_DIV16;   //定时器0进行1/16分割  (4<<0)->0100

    __raw_writel(tcfg1, S3C_TCFG1);  //将tcfg1的值写到分割寄存器中
    __raw_writel(tcfg0, S3C_TCFG0);  //将tcfg0的值写到分频寄存器中

    clk_p = clk_get(NULL, "pclk");   //得到pclk
    pclk  = clk_get_rate(clk_p);
    tcnt  = (pclk/50/16)/freq;    //得到定时器的输入时钟，进而设置PWM的调制频率

    __raw_writel(tcnt, S3C_TCNTB(0));   //PWM脉宽调制的频率等于定时器的输入时钟
    __raw_writel(tcnt/2, S3C_TCMPB(0));   //占空比是50%

    tcon &= ~0x1f;
    tcon |= 0xb;        //禁用死区, 间隔模式开启, 逆变器关闭, 自动更新TCNTB0&TCMPB0, 开始定时器0
    __raw_writel(tcon, S3C_TCON);  把tcon的设置写到计数控制寄存器S3C_TCON中

        tcon &= ~2;         //clear manual update bit
    __raw_writel(tcon, S3C_TCON);
}//}}}
//{{{ pwm stop
void PWM_Stop( void )
{
    unsigned tmp;
    tmp = readl(S3C64XX_GPFCON);  //设置GPF14为输出
    tmp &= ~(0x3U << 28);
    writel(tmp, S3C64XX_GPFCON);  //设置GPF14为低电平，使蜂鸣器停止
}//}}}

//{{{   open
static int s3c6410_buzzer_open(struct inode *inode, struct file *file)
{
    if (!down_trylock(&lock))  //是否获得信号量。如果是，down_trylock(&lock)=0,否则非0。
        return 0;
    else
        return -EBUSY;   //返回错误信息：请求资源不可用。
}//}}}
//{{{ close
static int s3c6410_buzzer_close(struct inode *inode, struct file *file)
{
    up(&lock);
    return 0;
}//}}}
//{{{   unlocked_ioctl
//  参数:
//      cmd = 0, 停止PWM;
//      cmd = 1, 开启PWM, 并设置频率;
static long s3c6410_buzzer_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
    switch (cmd) {  //如果cmd=1，进入case  PWM_IOCTL_SET_FREQ
        case PWM_IOCTL_SET_FREQ:
            if (arg == 0) //如果设置的频率参数是0
                return -EINVAL; //返回错误信息，表示向参数传递了无效的参数
            PWM_Set_Freq(arg); //否则设置给定的频率
            break;

        case PWM_IOCTL_STOP:  //如果cmd=0，进入
        default:
            PWM_Stop();  //停止蜂鸣器
            break;
    }

    return 0;
}//}}}

//{{{ file_operations
static struct file_operations S3C6410_BUZZER_FOPS = {  //初始化设备的文件操作的结构体
    .owner          = THIS_MODULE,
    .open           = s3c6410_buzzer_open,
    .release        = s3c6410_buzzer_close,
    .unlocked_ioctl = s3c6410_buzzer_ioctl,
};//}}}
//{{{   struct miscdevice
static struct miscdevice misc = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = DEVICE_NAME,
    .fops = &S3C6410_BUZZER_FOPS,
};//}}}

//{{{ module init
static int __init s3c6410_buzzer_init(void)
{
    int ret;

    sema_init(&lock, 1);  //初始化一个互斥锁

    //注册一个misc设备
    ret = misc_register(&misc);
    if (ret < 0) {          // register failed.
        printk(DEVICE_NAME " can't register mics device - Buzzer Driver\n");
        return ret;
    }
    printk(DEVICE_NAME" initialized.\n");
    return 0;
}//}}}
//{{{ module exit
static void __exit s3c6410_buzzer_exit(void)
{
    // 卸载驱动程序
    misc_deregister(&misc);

    printk(DEVICE_NAME " module exit.\n");
}//}}}
module_init(s3c6410_buzzer_init);
module_exit(s3c6410_buzzer_exit);
// {{{ Module Description
// 描述驱动程序的一些信息，不是必须的
MODULE_VERSION("0.0.1");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("SeanXP#iseanxp+code@gmail.com");
MODULE_DESCRIPTION("Tiny6410 Buzzer Driver");
// }}}
