from multiprocessing.reduction import sendfds
from re import T
from smtplib import SMTPServerDisconnected
from statistics import geometric_mean
import RPi.GPIO as GPIO 
import smbus #调用smbus库实现I2C通信协议
import math 
import time



# 定义 HMC5883L 寄存器地址
HMC5883L_ADDRESS = 0x1E
HMC5883L_MODE_REGISTER = 0x02
HMC5883L_DATA_REGISTER = 0x03

# 打开 I2C 总线
bus = smbus.SMBus(1)

# 设置 HMC5883L 工作模式
bus.write_byte_data(HMC5883L_ADDRESS, HMC5883L_MODE_REGISTER, 0)

# 磁场矫正
# 实验实际的磁场矫正参数，即靠近强磁铁时的最大参数
mx_min, mx_max = -61400, 61400
my_min, my_max = -61400, 61400
mz_min, mz_max = -61400, 61400
def calibrate(mx_raw, my_raw):
    mx = 2 * (mx_raw - mx_min) / (mx_max - mx_min) - 1
    my = 2 * (my_raw - my_min) / (my_max - my_min) - 1
    return mx, my

# 根据x、y轴磁感应强度计算x轴与北极的夹角
def calc_heading(mx, my):
    if mx > 0 and my > 0 :
        return math.atan(my/mx)
    elif mx >0 and my <0 :
        return 2*math.pi+math.atan(my/mx)
    elif mx < 0 and my >= 0:
        return math.atan(my/mx) + math.pi
    elif mx < 0 and my < 0:
        return math.atan(my/mx) + math.pi
    elif mx == 0 and my > 0:
        return math.pi / 2
    else:
        return -math.pi / 2

#读取hmc5883l的数据获取与北极的夹角
def drift_angle():
    # 读取 HMC5883L 数据
    data = bus.read_i2c_block_data(HMC5883L_ADDRESS, HMC5883L_DATA_REGISTER, 6)
    #将8位2进制数转化为10进制并处理符号位
    x = (data[0] * 256 + data[1])
    if x > 32767 :
        x -= 65536
    y = (data[4] * 256 + data[5])
    if y > 32767 :
        y -= 65536
    #校正偏差
    x0,y0=calibrate(x,y)
    #计算偏角
    dr=calc_heading(x0,y0)
    print("当前偏角：",dr/math.pi*180)
    return dr/math.pi*180



#以下为GPIO引脚地址
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
Motor1A = 11
Motor1B = 12
Motor1E = 16 	#使能通道A

Motor2A = 13
Motor2B = 15
Motor2E = 18 	#使能通道B
Sensor_L = 21   #左红外
Sensor_R = 22	#右红外

#print("Setting up GPIO pins")
GPIO.setup(Motor1A, GPIO.OUT)
GPIO.setup(Motor1B, GPIO.OUT)
GPIO.setup(Motor1E, GPIO.OUT)
GPIO.setup(Motor2A, GPIO.OUT)
GPIO.setup(Motor2B, GPIO.OUT)
GPIO.setup(Motor2E, GPIO.OUT)
GPIO.setup(Sensor_L,GPIO.IN)
GPIO.setup(Sensor_R,GPIO.IN)

#print("Warming up engines")
motorR = GPIO.PWM(Motor1E,100) 
motorL = GPIO.PWM(Motor2E,100)	
#print("Starting motors")


#以下为小车驱动代码
def initStart():
	motorR.start(0)
	motorL.start(0)

def exitAndClean():
	print("Exiting")
	motorR.stop()
	motorL.stop()
	GPIO.cleanup()
	exit()

def cleanNoExit():
	print("Exiting")
	motorR.stop()
	motorL.stop()
	GPIO.cleanup()

#停止
def stop():
	print("Stoping")
	GPIO.output(Motor1A, False)
	GPIO.output(Motor1B, False)
	GPIO.output(Motor2A, False)
	GPIO.output(Motor2B, False)
	motorR.ChangeDutyCycle(0)
	motorL.ChangeDutyCycle(0)

#前、后直行
def go(speed):#以speed参数的符号决定前、后
	if speed >= 0:#前进
		GPIO.output(Motor1A, GPIO.LOW)	
		GPIO.output(Motor1B, GPIO.HIGH)
		GPIO.output(Motor2A, GPIO.HIGH)
		GPIO.output(Motor2B, GPIO.LOW)
		motorR.ChangeDutyCycle(speed)
		motorL.ChangeDutyCycle(speed)
	elif speed < 0:#后退
		GPIO.output(Motor1A, GPIO.HIGH)
		GPIO.output(Motor1B, GPIO.LOW)
		GPIO.output(Motor2A, GPIO.LOW)
		GPIO.output(Motor2B, GPIO.HIGH)
		motorR.ChangeDutyCycle(speed)
		motorL.ChangeDutyCycle(speed)

#左转、右转
def turn(speed):#以speed参数的符号决定左、右
	if speed >=0:#右转
		GPIO.output(Motor1A, GPIO.HIGH)
		GPIO.output(Motor1B, GPIO.LOW)
		GPIO.output(Motor2A, False)
		GPIO.output(Motor2B, False)
		motorR.ChangeDutyCycle(speed)
		motorL.ChangeDutyCycle(speed)
	else:#左转
		GPIO.output(Motor1A, False)
		GPIO.output(Motor1B, False)
		GPIO.output(Motor2A, GPIO.LOW)
		GPIO.output(Motor2B, GPIO.HIGH)
		motorR.ChangeDutyCycle(-speed)
		motorL.ChangeDutyCycle(-speed)

#以下为双红外检测模块
def Senser_status():
	SL=GPIO.input(Sensor_L)
	SR=GPIO.input(Sensor_R)
	if SR != False and SL !=False: #无障碍
		return 0
	if SR == False and SL !=False:#右方障碍
		return 1
	if SR != False and SL ==False:#左方障碍
		return 2
	else :#前方障碍
		return 3	

#以下为以hmc5883l为基准实现左转、右转和回正
def turn_left(dr,tspeed):
	dr0=drift_angle()
	dr1=0
	if dr0 < dr: #0度减小到360度的情况
		while dr1 < dr:  
			print("！！！正在左转！！！")
			turn(tspeed)
			dr1=drift_angle()
			if dr1 > 270:
				dr1=360-dr1+dr0
			else:
				dr1=dr0-dr1
			time.sleep(0.01)
	else:
		while dr1 < dr:  
			print("！！！正在左转！！！")
			turn(tspeed)
			dr1=drift_angle()
			dr1=dr0-dr1
			time.sleep(0.01)

def turn_right(dr,tspeed):
	dr0=drift_angle()
	dr1=0
	if 360-dr0 < dr: #360度增加到0度的情况
		while dr1 < dr:  
			print("！！！正在右转！！！")
			turn(-tspeed)
			dr1=drift_angle()
			if dr1 < 90:
				dr1=360-dr0+dr1
			else:
				dr1-=dr0
			time.sleep(0.01)
	else:
		while dr1 < dr:  
			print("！！！正在右转！！！")
			turn(-tspeed)
			dr1=drift_angle()
			dr1-=dr0
			time.sleep(0.01)

def recover(dr,tspeed):#以符号判断最终回正的方向
	if tspeed >=0:
		turn_left(dr,tspeed)
	else:
		turn_right(dr,-tspeed)


#以下为小车自动避障模块
def RoundL(tspeed,dr,gspeed,gtime):#tspeed转速，gspeed前速,ttime转时，gtime前时

	if tspeed >=0:
		ldr=0 #初始偏移量
		turn_left(dr,tspeed)#左转绕过阶段
		sensor=Senser_status()
		start=time.perf_counter()
		initStart()
		while sensor==0: #无障碍
			end=time.perf_counter()
			if end-start > gtime:
				break
			sensor=Senser_status()
			go(gspeed)
		ldr+=dr*(end-start)
		if sensor!=0:
			gtime=ldr/dr

		turn_right(2*dr,tspeed) #右转回正阶段
		sensor=Senser_status()
		start=time.perf_counter()
		initStart()
		while sensor==0: #无障碍
			end=time.perf_counter()
			if end-start > gtime:
				break
			sensor=Senser_status()
			go(gspeed)
		ldr-=dr*(end-start)
		gtime-=ldr/dr#剩余右转时间
		if sensor!=0:
			initStart()
			turn_left(dr,tspeed)
			go(gspeed)
			time.sleep(gtime)
			initStart()
			turn_right(2*dr,tspeed)
			go(gspeed)
			time.sleep(2*gtime)
		recover(dr,tspeed)
	if tspeed <0:
		ldr=0 #初始偏移量
		turn_right(dr,-tspeed)#右转绕过阶段
		sensor=Senser_status()
		start=time.perf_counter()
		initStart()
		while sensor==0: #无障碍
			end=time.perf_counter()
			if end-start > gtime:
				break
			sensor=Senser_status()
			go(gspeed)
		ldr+=dr*(end-start)#剩余左转时间
		if sensor!=0:
			gtime=ldr/dr

		turn_left(2*dr,-tspeed) #左转回正阶段
		sensor=Senser_status()
		start=time.perf_counter()
		initStart()
		while sensor==0: #无障碍
			end=time.perf_counter()
			if end-start > gtime:
				break
			sensor=Senser_status()
			go(gspeed)
		ldr-=dr*(end-start)
		gtime-=ldr/dr#剩余左转时间
		if sensor!=0:
			initStart()
			turn_right(dr,-tspeed)
			go(gspeed)
			time.sleep(gtime)
			initStart()
			turn_left(2*dr,-tspeed)
			go(gspeed)
			time.sleep(2*gtime)
		recover(dr,tspeed)

def main():
	initStart()
	try:
		while True:
			go(60)
			sensor=Senser_status()
			if sensor==1:#右方障碍
				RoundL(70,20,100,3)
			elif sensor==2:#左方障碍
				RoundL(-70,20,100,3)
			elif sensor==3:#前方障碍
				stop()
				go(-100)
				time.sleep(1)
				RoundL(70,45,100,3)
	except KeyboardInterrupt: 
		GPIO.cleanup()

if __name__=="__main__":
	main()