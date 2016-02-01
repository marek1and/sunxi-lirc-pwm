LIRC RX/TX kernel driver for Cubietruck (Allwinner A20)

Device driver for LIRC using Allwinner A1X or A20 IR module in CIR mode for receiving and PWM output for transmitting IR signal.

Driver was tested on a Cubietruck with Allwinner A20 with linux-sunxi kernel (3.4.x).

A lot of code for receiving part was reused from [sunxi-lirc driver](https://github.com/matzrh/sunxi-lirc).
LIRC require space as a first value in buffer to correctly sync with received signal so additional code was added to RX driver part.

Transmission of IR signal is done by PWM output. By default PWM0 is used but this can be changed by setting driver pwm_channel parameter. For example:

<code>
	modprobe sunxi-lirc-pwm pwm_channel=1
</code>

IR Diode connection to Cubietruck was presented below, but you can simplify this by connecting IR diode via resistor directly to PWM output port.
![IR LED Connection Schema](https://raw.github.com/marek1and/sunxi-lirc-pwm/master/ir_tx.png)

To compile this module you need to correctly set up paths in Makefile:
<table>
	<tr>
		<td>KDIR</td><td>Cubietruck kernel source directory</td>
	</tr>
	<tr>
		<td>CROSS_COMPILE</td><td>gcc prefix to cross compile kernel driver</td>
	</tr>
</table>

After setting up needed variables you can simply type:

<code>
	make
</code>

and if compilation will succeed, then you should have sunxi-lirc-pwm.ko file in build directory.
