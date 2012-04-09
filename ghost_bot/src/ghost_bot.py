#!/usr/bin/python
import roslib; roslib.load_manifest("nxt_terminator")
import rospy
import nxt.locator
import curses
from nxt.telegram import Telegram

RIGHT = 'right'
LEFT = 'left'
SAFE_DISTANCE_CM = 40
STOP_DISTANCE_CM = 20
SLOPE = (100 / (SAFE_DISTANCE_CM - STOP_DISTANCE_CM))
YINTERSECT = SLOPE * STOP_DISTANCE_CM * -1 

keyWheelSpeeds = {
			curses.KEY_LEFT:(-100, 100),
			curses.KEY_RIGHT:(100, -100),
			curses.KEY_UP:(100, 100),
			curses.KEY_DOWN:(-100, -100),
			ord('s'): (0,0)
		}

def getKeyboardInput(stdscr):
	# do not wait for input when calling getch
	stdscr.nodelay(1)
	while True:
		# get keyboard input, returns -1 if none available
		c = stdscr.getch()
		if c != -1:
			# return curser to start position
			stdscr.move(0, 0)
			return c
		else:
			return ""

def setKinectData(key, kinectData):
	try:
		speeds = keyWheelSpeeds[key]  
		kinectData[LEFT] = speeds[0]
		kinectData[RIGHT] = speeds[1]
	except KeyError:
		pass 

def adjustMotorData(kinectData, sensor, stdscr):
	cur_distance = sensor.get_sample() 
	if cur_distance > SAFE_DISTANCE_CM:
		return
	else:
		if kinectData[RIGHT] >  0:
			kinectData[RIGHT] *= (max(((cur_distance * SLOPE) + YINTERSECT), 0) * 0.1) 
		if kinectData[LEFT] > 0:
			kinectData[LEFT] *= (max(((cur_distance * SLOPE) + YINTERSECT), 0) * 0.1)

def motor_control(stdscr, b):
	rospy.init_node('motor_control')

	kinectData = {RIGHT:0, LEFT:0} 

	while(True):
		key = getKeyboardInput(stdscr)
		if key:
			if key == curses.KEY_ENTER or key == ord('x'):
				break
			else:
				setKinectData(key, kinectData)

		adjustMotorData(kinectData, ultraSonic, stdscr)
		rightWheel.update(kinectData[RIGHT], 0)
		leftWheel.update(kinectData[LEFT], 0)

	rightWheel.stop()
	leftWheel.stop()

def blue_messenger(brick, message):
	tgram = Telegram(True, 0x09, False)
	tgram.add_u8(0)
	tgram.add_u8(len(message) + 1)
	tgram.add_string(len(message), message)
	tgram.add_u8(0)
	brick.sock.send(str(tgram))

def blue_dispatcher(stdscr, brick):
	while(True):
		key = getKeyboardInput(stdscr)
		if key:
			if key == curses.KEY_ENTER or key == ord('x'):
				break
			else:
				blue_messenger(brick, chr(key))	

def main(stdscr):
	sock = nxt.locator.find_one_brick()
	if sock:
		blue_dispatcher(stdscr, sock.connect())
		sock.close()
	else:
		print 'No NXT bricks found'

if __name__== "__main__":
	curses.wrapper(main)
