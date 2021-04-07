
#include <linux/init.h> 
#include <linux/semaphore.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <plat/dmtimer.h>

struct gptimer_pwm_dev {
	int id;
	struct omap_dm_timer *timer;
	struct semaphore sem;
	u32 input_freq;
	u32 tldr;
	u32 tmar;
	u32 num_settings;
	u32 current_val;
	u32 set;
};

static struct gptimer_pwm_dev gptimer9_pwm_dev;

static int frequency;

static void __init gptimer9_pwm_set_frequency(struct gptimer_pwm_dev *pd)
{
	if (frequency > (pd->input_freq / 2)) 
		frequency = pd->input_freq / 2;

	pd->tldr = 0xFFFFFFFF - ((pd->input_freq / frequency) - 1);

	omap_dm_timer_set_load(pd->timer, 1, pd->tldr);

	pd->num_settings = 0xFFFFFFFE - pd->tldr;
}

int gptimer9_pwm_set_duty_cycle(u32 duty_cycle) 
{
	u32 new_tmar;

	if(gptimer9_pwm_dev.set !=  1)
	{
		printk("%s:pwm_init fail or not executed.\n",__FUNCTION__);
		return -EINVAL;
	}

	if (duty_cycle > 100)
		return -EINVAL;

	if (down_interruptible(&gptimer9_pwm_dev.sem))
		return -ERESTARTSYS;

	if (duty_cycle == 0) {
		omap_dm_timer_stop(gptimer9_pwm_dev.timer);
		gptimer9_pwm_dev.current_val = 0;
		goto releas_sem_exit;
	}
 
	new_tmar = (duty_cycle * gptimer9_pwm_dev.num_settings) / 100;

	if (new_tmar < 1) 
		new_tmar = 1;
	else if (new_tmar > gptimer9_pwm_dev.num_settings)
		new_tmar = gptimer9_pwm_dev.num_settings;
		
	gptimer9_pwm_dev.tmar = gptimer9_pwm_dev.tldr + new_tmar;

	omap_dm_timer_set_match(gptimer9_pwm_dev.timer, 1, gptimer9_pwm_dev.tmar);

	if (gptimer9_pwm_dev.current_val == 0)
		omap_dm_timer_start(gptimer9_pwm_dev.timer);

	gptimer9_pwm_dev.current_val = duty_cycle;

releas_sem_exit:

	up(&gptimer9_pwm_dev.sem);

	return 0;
}


static void gptimer9_pwm_timer_cleanup(void)
{

	if (gptimer9_pwm_dev.timer) {
		omap_dm_timer_free(gptimer9_pwm_dev.timer);
		gptimer9_pwm_dev.timer = NULL;
		gptimer9_pwm_dev.set = 0;
	}
}

static int __init gptimer9_pwm_timer_init(void)
{
	struct clk *fclk;

	gptimer9_pwm_dev.timer = omap_dm_timer_request_specific(gptimer9_pwm_dev.id);

	if (!gptimer9_pwm_dev.timer)
			goto timer_init_fail;

	omap_dm_timer_set_pwm(gptimer9_pwm_dev.timer,
			0,	// ~SCPWM low when off
			1,	// PT pulse toggle modulation
			OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);

	if (omap_dm_timer_set_source(gptimer9_pwm_dev.timer, 
					OMAP_TIMER_SRC_SYS_CLK))
		goto timer_init_fail;

	// make sure we know the source clock frequency
	fclk = omap_dm_timer_get_fclk(gptimer9_pwm_dev.timer);
	gptimer9_pwm_dev.input_freq = clk_get_rate(fclk);

	gptimer9_pwm_set_frequency(&gptimer9_pwm_dev);

	gptimer9_pwm_dev.set = 1;

	return 0;
	
timer_init_fail:

	gptimer9_pwm_timer_cleanup();
	
	return -1;
}

int __init gptimer9_pwm_init(void)
{
	gptimer9_pwm_dev.id = 9;

	sema_init(&gptimer9_pwm_dev.sem, 1);
	
	frequency = 1024;

	if (gptimer9_pwm_timer_init())
		goto init_fail_2;

	printk(KERN_INFO "pwm: frequency=%d Hz\n", frequency);
	
	return 0;
	
init_fail_2:	
	gptimer9_pwm_timer_cleanup();
	
	return -1;
}
