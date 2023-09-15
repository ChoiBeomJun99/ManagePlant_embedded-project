#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <asm/delay.h>
#include <linux/cdev.h>
#include <linux/timer.h>



#define DEV_NAME "proj_ioctl_dev"
#define IOCTL_START_NUM 0x80
#define IOCTL_NUM1 IOCTL_START_NUM+1
#define IOCTL_NUM2 IOCTL_START_NUM+2

#define PROJ_IOCTL_NUM 'z'
#define PROJ_IOCTL1 _IOWR(PROJ_IOCTL_NUM, IOCTL_NUM1, unsigned long)
#define PROJ_IOCTL2 _IOWR(PROJ_IOCTL_NUM, IOCTL_NUM2, unsigned long*)

spinlock_t my_lock;

MODULE_LICENSE("GPL");
// for stepper motor

#define PIN1 6
#define PIN2 13
#define PIN3 19
#define PIN4 26
#define STEPS 8
#define ONEROUND 128

int blue[8] = {1, 1, 0, 0, 0, 0, 0, 1};
int pink[8] = {0, 1, 1, 1, 0, 0, 0, 0};
int yellow[8] = {0, 0, 0, 1, 1, 1, 0, 0};
int orange[8] = {0,0,0,0,0,1,1,1,};

int sensorResultTime = 3000;

//for temp_humidity_sensor

#define MAX_TIMING 85
#define DHT11 21

static int temp_supdo_data[5] = {0, };

//for ultra_sonic_detector

#define LED1 12
#define ULTRA_TRIG 17
#define ULTRA_ECHO 18

int cm = -1;
static int irq_num1;
static int echo_valid_flag = 3;

static ktime_t echo_start;
static ktime_t echo_stop;



//for timer

struct my_timer_info {
    struct timer_list timer;
    long delay_jiffies;
    int data;
};

static struct my_timer_info my_timer;


void setstep(int p1, int p2, int p3, int p4) {
	gpio_set_value(PIN1, p1);
	gpio_set_value(PIN2, p2);
	gpio_set_value(PIN3, p3);
	gpio_set_value(PIN4, p4);
}

void backward(int round, int delay) {
	int i = 0, j = 0;

	for (i =0; i<ONEROUND * round; i++) {
		for (j = STEPS; j>0; j--) {
			setstep(blue[j], pink[j], yellow[j], orange[j]);
			udelay(delay);
		}
	}
	setstep(0,0,0,0);
}

void forward (int round, int delay) {
	int i =0, j=0;

	for(i=0; i< ONEROUND * round; i++) {
		for(j=0;j<STEPS;j++) {
			setstep(blue[j], pink[j], yellow[j], orange[j]);
			udelay(delay);
		}
	}

	setstep(0,0,0,0);
}

void waterToPlant (int time) {
	printk("motor on\n");
	forward(1,1500);
	mdelay(time);
	backward(1,1500);
}

static int dht11_read(void){
    int last_state = 1;
    int counter = 0;
    int i = 0; int j = 0;

    temp_supdo_data[0] = temp_supdo_data[1] = temp_supdo_data[2] = temp_supdo_data[3] = temp_supdo_data[4] = 0;

    gpio_direction_output(DHT11, 0);
    gpio_set_value(DHT11, 0);
    mdelay(18);
    gpio_set_value(DHT11, 1);
    udelay(40);
    gpio_direction_input(DHT11);

    for(i = 0; i < MAX_TIMING; i++){
        counter = 0;
        while (gpio_get_value(DHT11) == last_state){
            counter++;
            udelay(1);
            if(counter == 255){
                break;
            }
        }
        last_state = gpio_get_value(DHT11);

        if (counter == 255){
            break;
        }

        if((i>=4) && (i%2 ==0)){
            temp_supdo_data[j/8] <<= 1;
            if(counter > 16){
                temp_supdo_data[j/8] |= 1;
            }
            j++;
        }
    }
    //check sum
    if ((j>=40) &&
        (temp_supdo_data[4] == ( (temp_supdo_data[0] + temp_supdo_data[1] + temp_supdo_data[2] + temp_supdo_data[3]) & 0xFF))){
        int sec = 0;
        printk("Humdity : %d.%d Temperature = %d.%d C\n", temp_supdo_data[0], temp_supdo_data[1], temp_supdo_data[2], temp_supdo_data[3]);
        if(temp_supdo_data[0] < 30) {
            sec += 3;
        }
        else if (temp_supdo_data[0] >= 30 && temp_supdo_data[0] < 35) {
            sec += 2;
        }
        else if (temp_supdo_data[0] >= 35) {
            sec += 1;
        }

        if(temp_supdo_data[2] < 20) {
            sec += 1;
        } else if (temp_supdo_data[2] >=20 && temp_supdo_data[2] < 25) {
            sec += 2;
        } else if (temp_supdo_data[2] >=25 && temp_supdo_data[2] < 30) {
            sec += 3;
        } else if (temp_supdo_data[2] >=30 && temp_supdo_data[2] < 35) {
            sec += 4;
        } else if (temp_supdo_data[2] >= 35) { 
            sec += 5;
        }

        return sec;
    }else{
        printk("Data not good, skip\n");
        return -1;
    }
}


static irqreturn_t simple_ultra_isr(int irq, void* dev_id) {
	
	ktime_t tmp_time;
	s64 time;
	
	// start time
	tmp_time = ktime_get();
	
	// ultra sonic on
	if(echo_valid_flag == 1) {
		printk("simple_ultra: Echo up\n");
		if(gpio_get_value(ULTRA_ECHO) == 1) {
			echo_start = tmp_time;
			echo_valid_flag = 2;
		}
	} 
	// ultra sonic back
	else if(echo_valid_flag == 2) {
        spin_lock(&my_lock);
		printk("simple_ultra: Echo down\n");
		if(gpio_get_value(ULTRA_ECHO) == 0) {
			echo_stop = tmp_time;
			time = ktime_to_us(ktime_sub(echo_stop, echo_start));
			cm = (int) time / 58;
			printk("simple_ultra: Detect %d cm\n", cm);
			
			// not enough water
			if(cm > 5) {
				gpio_set_value(LED1,1);
			} 
			// enough water - case of refilled
			else {
				
				gpio_set_value(LED1,0);

				// motor on
				//
			}

			echo_valid_flag = 3;
		}
        spin_unlock(&my_lock);
	}

	return IRQ_HANDLED;
}

void check_water(void){
    if(echo_valid_flag == 3) {
        echo_start = ktime_set(0,1);
        echo_stop = ktime_set(0,1);
        echo_valid_flag = 0;

        gpio_set_value(ULTRA_TRIG,1);
        udelay(10);
        gpio_set_value(ULTRA_TRIG,0);
    
        echo_valid_flag = 1;
    }
}


static void timer_water_func(struct timer_list *t){
    int sec = -1;
    while(sec == -1){
        sec = dht11_read();
    }
    // printk("%d" , sec);
    waterToPlant(sec*1000);
}


static int proj_ioctl_open(struct inode *inode, struct file *file){
    return 0;   
}

static int proj_ioctl_release(struct inode *inode, struct file *file){
    return 0; 
}

static long proj_ioctl(struct file *file, unsigned int cmd, unsigned long arg){
	unsigned long flags;
	int sec;
	int msecs;

	switch(cmd){
        case PROJ_IOCTL1:
	    sec = -1;
	    printk("in PROJ_IOCTL1\n");
            // unsigned long flags;

            check_water();
            mdelay(1000);

            spin_lock_irqsave(&my_lock, flags);
            if(cm <= 5){
                while(sec == -1){
                    sec = dht11_read();
                }
                printk("%d" , sec);
                waterToPlant(sec*1000);
            } else if (cm == -1)  {    
                printk("no interrupt");
            }
            spin_unlock_irqrestore(&my_lock, flags);
	    
	    break;
        case PROJ_IOCTL2:
	    printk("in ioctl PROJ_IOCTL2\n");
            msecs = (int)arg * 1000 * 60;
            mod_timer(&my_timer.timer, jiffies + msecs_to_jiffies(msecs));

	    break;
        default:
	    // error
            return -1;
    }
    return 0;
}


struct file_operations proj_ioctl_fops = {
  .open = proj_ioctl_open,
  .release = proj_ioctl_release,
  .unlocked_ioctl = proj_ioctl,
};

static dev_t dev_num;
static struct cdev *cd_cdev;



static int __init proj_init(void){
    int ret;
    // spinlock
    spin_lock_init(&my_lock);

    gpio_request_one(PIN1, GPIOF_OUT_INIT_LOW, "p1");
	gpio_request_one(PIN2, GPIOF_OUT_INIT_LOW, "p2");
	gpio_request_one(PIN3, GPIOF_OUT_INIT_LOW, "p3");
	gpio_request_one(PIN4, GPIOF_OUT_INIT_LOW, "p4");
    gpio_request(DHT11, "DHT11");
    gpio_request_one(ULTRA_TRIG, GPIOF_OUT_INIT_LOW, "ULTRA_TRIG");
	gpio_request_one(ULTRA_ECHO, GPIOF_IN, "ULTRA_ECHO");
    gpio_request_one(LED1, GPIOF_OUT_INIT_LOW, "LED1");
    timer_setup(&my_timer.timer, timer_water_func, 0);
    irq_num1 = gpio_to_irq(ULTRA_ECHO);
	ret = request_irq(irq_num1, simple_ultra_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "ULTRA_ECHO", NULL);
	if(ret) {
		printk("unable to request IRQ %d\n", ret);
		free_irq(irq_num1, NULL);
	}
	else {
		// handled
		
	}

    alloc_chrdev_region(&dev_num, 0 , 1, DEV_NAME);
    cd_cdev = cdev_alloc();
    cdev_init(cd_cdev, &proj_ioctl_fops);
    cdev_add(cd_cdev, dev_num, 1);

    // spin_unlock_irqrestore(&my_lock, flags);
    // check_water();
    return 0;
}

static void __exit proj_exit(void){
    del_timer(&my_timer.timer);
    gpio_free(PIN1);
	gpio_free(PIN2);
	gpio_free(PIN3);
	gpio_free(PIN4);
    gpio_free(LED1);
    gpio_set_value(DHT11 ,0);
    gpio_free(DHT11);
    free_irq(irq_num1, NULL);
    cdev_del(cd_cdev);
    unregister_chrdev_region(dev_num, 1);
}

module_init(proj_init);
module_exit(proj_exit);


