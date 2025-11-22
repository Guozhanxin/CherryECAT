/*
 * Copyright (c) 2025, sakumisu
 * Copyright (c) 2025, RuiChing
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <netdev.h>
#include "ec_master.h"

/* 网卡设备名称 */
#define ENET_DEVICE_NAME "e1"
/* 设置裸驱回调函数的命令码 */
#define SET_CALLBACK 0x10

/* 网络设备全局实例 */
ec_netdev_t g_netdev;
/* 网络设备句柄 */
static rt_device_t net_dev = RT_NULL;

/* 发送缓冲区 */
__attribute__((__aligned__(32))) 
static uint8_t tx_buffer[CONFIG_EC_MAX_ENET_TXBUF_COUNT][1536];
/* 接收缓冲区 - 适当对齐以提高性能 */
__attribute__((__aligned__(32)))
static uint8_t rx_buffer[CONFIG_EC_MAX_ENET_RXBUF_COUNT][1536];
static uint8_t current_rx_buf_idx = 0;

/* 接收回调函数 - 在中断上下文中被调用 */
static void enet_recv_callback(void *dev)
{
    rt_ssize_t len;
    
    /* 从设备读取数据 */
    while(1) {
        /* 循环使用接收缓冲区 */
        len = rt_device_read(net_dev, 0, rx_buffer[current_rx_buf_idx], sizeof(rx_buffer[0]));
        
        if (len > 0) {
            
            /* 检查是否为EtherCAT帧 - EtherCAT协议类型为0x88a4 */
            if (len >= 14 && *(uint16_t *)(&rx_buffer[current_rx_buf_idx][12]) == ec_htons(0x88a4)) {
                /* 处理EtherCAT帧 */
                ec_netdev_receive(&g_netdev, rx_buffer[current_rx_buf_idx], len);
            } else {
                /* 非EtherCAT帧，可选择忽略或记录 */
                EC_LOG_DBG("Received non-EtherCAT frame (type: 0x%04x)\n", 
                           *(uint16_t *)(&rx_buffer[current_rx_buf_idx][12]));
            }
            
            /* 更新接收缓冲区索引 */
            current_rx_buf_idx++;
            current_rx_buf_idx %= CONFIG_EC_MAX_ENET_RXBUF_COUNT;
        }
        else {
            break;
        }
    }
}

/* 初始化网络设备 */
ec_netdev_t *ec_netdev_low_level_init(uint8_t netdev_index)
{
    rt_base_t level;
    uint8_t mac[6] = {0};

    /* 查找网络设备 */
    net_dev = rt_device_find(ENET_DEVICE_NAME);
    if (net_dev == RT_NULL) {
        EC_LOG_ERR("Find network device %s failed!\n", ENET_DEVICE_NAME);
        return NULL;
    }
    
    /* 打开网络设备 */
    if (rt_device_open(net_dev, RT_DEVICE_FLAG_RDWR) != RT_EOK) {
        EC_LOG_ERR("Open network device %s failed!\n", ENET_DEVICE_NAME);
        return NULL;
    }
    
    /* 获取MAC地址 */
    if (if_get_mac(ENET_DEVICE_NAME, mac) != 0) {
        EC_LOG_ERR("Get MAC address failed!\n");
        rt_device_close(net_dev);
        return NULL;
    }
    
    /* 保存MAC地址到网络设备结构 */
    ec_memcpy(g_netdev.mac_addr, mac, 6);
    /* 初始化发送缓冲区的EtherCAT帧头 */
    for (uint32_t i = 0; i < CONFIG_EC_MAX_ENET_TXBUF_COUNT; i++) {
        for (uint8_t j = 0; j < 6; j++) { /* 目标MAC地址 - EtherCAT使用广播地址 */
            EC_WRITE_U8(&tx_buffer[i][j], 0xFF);
        }
        for (uint8_t j = 0; j < 6; j++) { /* 源MAC地址 */
            EC_WRITE_U8(&tx_buffer[i][6 + j], mac[j]);
        }
        EC_WRITE_U16(&tx_buffer[i][12], ec_htons(0x88a4)); /* EtherCAT协议类型 */
    }
    
    /* 设置裸驱回调函数 */
    level = rt_hw_interrupt_disable();
    if (rt_device_control(net_dev, SET_CALLBACK, enet_recv_callback) != RT_EOK) {
        rt_hw_interrupt_enable(level);
        EC_LOG_ERR("Set raw driver callback failed!\n");
        rt_device_close(net_dev);
        return NULL;
    }
    rt_hw_interrupt_enable(level);
    
    EC_LOG_DBG("Network device %s initialized successfully!\n", ENET_DEVICE_NAME);
    return &g_netdev;
}

static bool link_state = true;
static bool previous_link_state = false;

/* 检查链路状态 */
void ec_netdev_low_level_poll_link_state(ec_netdev_t *netdev)
{
#define GET_LINK_STATE      0x20
    
    rt_device_control(net_dev, GET_LINK_STATE, &link_state);

    netdev->link_state = link_state;
}

/* 命令行工具，用于手动设置链路状态（仅用于测试） */
static int cmd_link_up(int argc, char **argv)
{
    link_state = true;
    EC_LOG_INFO("Manual link up set\n");
    return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_link_up, link_up, configure netdev link up);

static int cmd_link_down(int argc, char **argv)
{
    link_state = false;
    EC_LOG_INFO("Manual link down set\n");
    return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_link_down, link_down, configure netdev link down);


/* 获取发送缓冲区 */
EC_FAST_CODE_SECTION uint8_t *ec_netdev_low_level_get_txbuf(ec_netdev_t *netdev)
{
    return (uint8_t *)tx_buffer[netdev->tx_frame_index];
}

/* 发送数据 */
EC_FAST_CODE_SECTION int ec_netdev_low_level_output(ec_netdev_t *netdev, uint32_t size)
{
    rt_size_t sent_len;
    rt_base_t level;
    
    /* 确保大小不超过MTU */
    if (size > 1536) {
        EC_LOG_ERR("Packet size %d exceeds MTU 1536\n", size);
        return -1;
    }
    
    /* 调用设备写接口发送数据 */
    /* 关闭中断以避免竞态条件 */
    level = rt_hw_interrupt_disable();
    sent_len = rt_device_write(net_dev, 0, tx_buffer[netdev->tx_frame_index], size);
    rt_hw_interrupt_enable(level);
    
    if (sent_len != size) {
        EC_LOG_ERR("Send data failed, expect %d, sent %d!\n", size, sent_len);
        return -1;
    }
    
    /* 更新发送帧索引 */
    netdev->tx_frame_index++;
    netdev->tx_frame_index %= CONFIG_EC_MAX_ENET_TXBUF_COUNT;
    
    return 0;
}

/* 接收数据 - 此函数在中断回调中已经处理，这里提供兼容性实现 */
EC_FAST_CODE_SECTION int ec_netdev_low_level_input(ec_netdev_t *netdev)
{
    /* 由于我们使用回调函数处理接收，这里简单返回0表示成功 */
    /* 实际项目中可以根据需要调整实现 */
    return 0;
}

/* 高精度定时器相关 */
static rt_device_t timer_dev = RT_NULL;
static ec_htimer_cb g_ec_htimer_cb = NULL;
static void *g_ec_htimer_arg = NULL;
static char timer_name[16] = "timer3"; /* 默认定时器名称 */

/* 定时器中断回调函数 */
static rt_err_t timer_callback(rt_device_t dev, rt_size_t size)
{
    if (g_ec_htimer_cb) {
        g_ec_htimer_cb(g_ec_htimer_arg);
    }
    return RT_EOK;
}

/* 启动高精度定时器 */
void ec_htimer_start(uint32_t us, ec_htimer_cb cb, void *arg)
{
    rt_hwtimer_mode_t mode = HWTIMER_MODE_PERIOD;
    rt_hwtimerval_t timeout = {0};
    rt_base_t level;
    
    /* 参数检查 */
    if (cb == NULL) {
        EC_LOG_ERR("Invalid timer callback\n");
        return;
    }
    
    /* 中断保护 */
    level = rt_hw_interrupt_disable();
    g_ec_htimer_cb = cb;
    g_ec_htimer_arg = arg;
    rt_hw_interrupt_enable(level);
    
    /* 尝试找到可用的硬件定时器 */
    timer_dev = rt_device_find(timer_name);
    if (timer_dev == RT_NULL) {
        EC_LOG_ERR("Cannot find timer device: %s\n", timer_name);
        return;
    }
    
    /* 打开定时器 */
    if (rt_device_open(timer_dev, RT_DEVICE_FLAG_RDWR) != RT_EOK) {
        EC_LOG_ERR("Open timer %s failed!\n", timer_name);
        return;
    }
    
    /* 设置回调函数 */
    rt_device_set_rx_indicate(timer_dev, timer_callback);
    
    /* 设置为周期性模式 */
    rt_device_control(timer_dev, HWTIMER_CTRL_MODE_SET, (void *)&mode);
    
    /* 设置超时时间 */
    timeout.sec = us / 1000000;
    timeout.usec = us % 1000000;

    if (rt_device_write(timer_dev, 0, &timeout, sizeof(timeout)) == 0) {
        EC_LOG_ERR("Timer write failed!\n");
        rt_device_close(timer_dev);
        timer_dev = RT_NULL;
        return;
    }
    
    EC_LOG_DBG("High precision timer started with %d us\n", us);
}

/* 停止高精度定时器 */
void ec_htimer_stop(void)
{
    rt_base_t level = rt_hw_interrupt_disable();
    if (timer_dev != RT_NULL) {
        rt_device_close(timer_dev);
        timer_dev = RT_NULL;
    }
    rt_hw_interrupt_enable(level);
}

/* 时间戳相关功能 */
static volatile rt_uint64_t ec_timestamp_overflow = 0;
static rt_uint32_t last_timer_value = 0;

/* 初始化时间戳功能 */
void ec_timestamp_init(void)
{
    /* 使用系统全局定时器初始化时间戳 */
    ec_timestamp_overflow = 0;
    last_timer_value = 0;
    
    /* 可以在这里添加额外的时间戳初始化代码 */
    EC_LOG_DBG("Timestamp functionality initialized\n");
}

/* 获取纳秒级时间戳 */
EC_FAST_CODE_SECTION uint64_t ec_timestamp_get_time_ns(void)
{
    rt_base_t level;
    uint32_t current_timer;
    uint64_t result;
    
    /* 读取当前定时器值，防止中断干扰 */
    level = rt_hw_interrupt_disable();
    current_timer = rt_hw_global_timer_get();
    
    /* 检测定时器溢出 */
    if (current_timer < last_timer_value) {
        ec_timestamp_overflow++;
    }
    last_timer_value = current_timer;
    rt_hw_interrupt_enable(level);
    
    /* 计算64位时间戳并转换为纳秒 */
    /* 假设24MHz时钟频率，每个时钟周期约41.666纳秒 */
    result = (ec_timestamp_overflow << 32) | current_timer;
    return (result * 1000) / 24;
}

/* 获取微秒级时间戳 */
EC_FAST_CODE_SECTION uint64_t ec_timestamp_get_time_us(void)
{
    rt_base_t level;
    uint32_t current_timer;
    uint64_t result;
    
    /* 读取当前定时器值，防止中断干扰 */
    level = rt_hw_interrupt_disable();
    current_timer = rt_hw_global_timer_get();
    
    /* 检测定时器溢出 */
    if (current_timer < last_timer_value) {
        ec_timestamp_overflow++;
    }
    last_timer_value = current_timer;
    rt_hw_interrupt_enable(level);
    
    /* 计算64位时间戳并转换为微秒 */
    /* 假设24MHz时钟频率，每个时钟周期约0.041666微秒 */
    result = (ec_timestamp_overflow << 32) | current_timer;
    return result / 24;
}

/* 清理函数 - 可用于关闭设备和释放资源 */
void ec_netdev_low_level_cleanup(ec_netdev_t *netdev)
{
    rt_base_t level;
    
    if (net_dev != RT_NULL) {
        EC_LOG_DBG("Cleaning up netdev resources\n");
        
        /* 停止定时器 */
        ec_htimer_stop();
        
        /* 取消回调设置 */
        level = rt_hw_interrupt_disable();
        rt_device_control(net_dev, SET_CALLBACK, RT_NULL);
        rt_hw_interrupt_enable(level);
        
        /* 关闭设备 */
        rt_device_close(net_dev);
        net_dev = RT_NULL;
    }
    
    /* 重置全局状态 */
    link_state = false;
    previous_link_state = false;
    current_rx_buf_idx = 0;
}
