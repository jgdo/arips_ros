#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <stdio.h>
#include <linux/soundcard.h>
#include <string.h>
#include <stdint.h>
#include <termios.h>
#include <time.h>
#include <errno.h>
#include <exception>
#include <boost/lexical_cast.hpp>
#include <iostream>

#include "utils.h"
#include "sysarips.h"

#include <ros/ros.h>

using namespace std;

static long long GetUsTicks() {
	timespec ts;
	if (clock_gettime(CLOCK_MONOTONIC, &ts) < 0)
		return -1;

	return ts.tv_nsec / 1000 + ts.tv_sec * 1000000LL;
}

SysArips::SysArips() {
	uartfd = -1;
	simulMode = false;

	currentDrivenDistLeft = currentDrivenDistRight = 0;
	currentDriveDist = 0;

	currentMotorLeft = currentMotorRight = 0;

	currentAngle = 0;
	currentPosX = currentPosY = 0;
	currentSpeed = currentAngleSpeed = 0;
	
	aripsCommand = -1;
	
	wheelDist = 0.325;

	for (int n = 0; n < 8; n++) {
		currentADC[n] = 0;
		currentServoPos[n] = 127;
	}
}

void SysArips::StartSimulation() {
	Close();
	simulMode = true;
}

int SysArips::Open() {
	simulMode = false;
	struct termios tio;

	const char *uartPath = getenv("ARIPS_UART_PATH");
	if (!uartPath)
		uartPath = "/dev/ttyUSB0";

	if ((uartfd = open(uartPath, O_RDWR)) < 0)
		return -1;

	tcgetattr(uartfd, &tio);
	cfsetospeed(&tio, B115200);
	cfsetispeed(&tio, B115200);
	cfmakeraw(&tio);
	tcsetattr(uartfd, TCSANOW, &tio);

	aripsCommand = -1;

	usleep(100000);

	ResetEncoders();

	usleep(30000);

	oldTime = GetUsTicks();

	return 0;
}

void SysArips::ProcessByte(int byte) {
	// cout << "ProcessByte " << (int)byte << endl;
	if (aripsCommand < 0) // command byte expected
	{
		if (IsCommand(byte)) // command byte
		{
			aripsCommand = byte;
			aripsParams.clear();
		}
	} else // parameters expected
	{
		if (byte == CMD_DROP) // drop command
			aripsCommand = -1;
		else if (byte == CMD_END) // command end
		{
			DoCommand();
			aripsCommand = -1;
		} else if (byte == CMD_SPACE) // spacing between parameters
			aripsParams.push_back(0);
		else if (byte >= 225) // parameter byte
		{
			if (aripsParams.empty()) // push a new parameter
				aripsParams.push_back(byte - 240);
			else // shift old parameter
				aripsParams.back() = (aripsParams.back() << 4) + (byte - 240);
		} else // command byte received
		{
			// ignore last command and start a new command line
			aripsCommand = byte;
			aripsParams.clear();
		}
	}
}

void SysArips::ProcessMotion(int dCountL, int dCountR) {
	static ros::Time lastTime = ros::Time::now();
	
    long long diffL = (dCountL - currentDrivenDistLeft);
    long long diffR = (dCountR - currentDrivenDistRight);
    
    ros::Time nowTime = ros::Time::now();
	if(diffL || diffR) {
		// http://www-home.fh-konstanz.de/~bittel/roboMSI/Vorlesung/04_Lokalisierung.pdf
		double theta = currentAngle;	

		// delta in meter
		double fl = diffL * ENCODER_FACTOR,
				fr = diffR * ENCODER_FACTOR;

		double d = (fl + fr) / 2;
		double alpha = (fr - fl) / wheelDist;	
		
		ros::Duration dt = nowTime - lastTime;
		currentSpeed = d / dt.toSec();
		currentAngleSpeed = alpha / dt.toSec();

		double x = currentPosX + d * cos(theta + alpha / 2); // neue posX	
		double y = currentPosY + d * sin(theta + alpha / 2); // neue posY	
		
		currentDrivenDistLeft += diffL;
		currentDrivenDistRight += diffR;
		currentDriveDist += (diffL + diffR) / 2;

		currentPosX = x;
		currentPosY = y;
		currentAngle += alpha;
	} else {
		currentSpeed = currentAngleSpeed = 0;
	}
	
	lastTime = nowTime;

    for (std::list < boost::function<void()> >::iterator iter = onNewPosition.begin(); iter != onNewPosition.end(); ++iter)	
        (*iter)();
}

void SysArips::Close() {
	if (uartfd >= 0) {
		DriveAt(0, 0);
		close(uartfd);
		uartfd = -1;
	}
}

void SysArips::ReadInput() {
	if (uartfd < 0)
		throw runtime_error("UART not opened");
	
	uint8_t buffer[1024];

	int r = read(uartfd, buffer, sizeof (buffer));
	if (r < 0) // error
		throw runtime_error("read failed with %d" + boost::lexical_cast<std::string>(errno));

	for (int i = 0; i < r; i++)
		ProcessByte(buffer[i]);
		//cout << "recv " << (int)buffer[i] << endl;
}

void SysArips::SendByte(int b) {
	if (uartfd < 0)
		throw runtime_error("UART not opened");
	if(write(uartfd, &b, 1) < 0)
		throw runtime_error("write failed withd" + boost::lexical_cast<std::string>(errno));
}

SysArips& SysArips::operator <<(uint8_t value) {
	char valueBuffer[12];

	if (value == 0)
		SendByte(240);
	else {
		int i = 0;
		do {
			valueBuffer[i++] = value & 0xF;
			value >>= 4;
		} while (value > 0);

		while (i > 0)
			SendByte(240 + valueBuffer[--i]);
	}

	return *this;
}

SysArips& SysArips::operator <<(int8_t value) {
	char valueBuffer[12];

	if (value < 0) {
		value *= -1;
		int i = 0;
		do {
			valueBuffer[i++] = value & 0xF;
			value >>= 4;
		} while (value > 0);

		while (i > 0)
			SendByte(240 - valueBuffer[--i]);
	} else if (value == 0)
		SendByte(240);
	else {
		int i = 0;
		do {
			valueBuffer[i++] = value & 0xF;
			value >>= 4;
		} while (value > 0);

		while (i > 0)
			SendByte(240 + valueBuffer[--i]);
	}

	return *this;
}

SysArips& SysArips::operator <<(uint16_t value) {
	char valueBuffer[12];

	if (value == 0)
		SendByte(240);
	else {
		int i = 0;
		do {
			valueBuffer[i++] = value & 0xF;
			value >>= 4;
		} while (value > 0);

		while (i > 0)
			SendByte(240 + valueBuffer[--i]);
	}

	return *this;
}

SysArips& SysArips::operator <<(int16_t value) {
	char valueBuffer[12];

	if (value < 0) {
		value *= -1;
		int i = 0;
		do {
			valueBuffer[i++] = value & 0xF;
			value >>= 4;
		} while (value > 0);

		while (i > 0)
			SendByte(240 - valueBuffer[--i]);
	} else if (value == 0)
		SendByte(240);
	else {
		int i = 0;
		do {
			valueBuffer[i++] = value & 0xF;
			value >>= 4;
		} while (value > 0);

		while (i > 0)
			SendByte(240 + valueBuffer[--i]);
	}

	return *this;
}

SysArips& SysArips::operator <<(uint32_t value) {
	char valueBuffer[12];

	if (value == 0)
		SendByte(240);
	else {
		int i = 0;
		do {
			valueBuffer[i++] = value & 0xF;
			value >>= 4;
		} while (value > 0);

		while (i > 0)
			SendByte(240 + valueBuffer[--i]);
	}

	return *this;
}

SysArips& SysArips::operator <<(int32_t value) {
	char valueBuffer[12];
	// printf("sending: ");
	if (value < 0) {
		value *= -1;
		int i = 0;
		do {
			valueBuffer[i++] = value & 0xF;
			value >>= 4;
		} while (value > 0);

		while (i > 0)
			SendByte(240 - valueBuffer[--i]);
	} else if (value == 0)
		SendByte(240);
	else {
		int i = 0;
		do {
			valueBuffer[i++] = value & 0xF;
			value >>= 4;
		} while (value > 0);

		while (i > 0) {
			int c = valueBuffer[--i];
			SendByte(240 + c);
			// printf("%d, ", c);
		}
	}

	// printf("\n");
	return *this;
}

SysArips& SysArips::operator <<(CommonCmd cmd) {
	SendByte(cmd);
	return *this;
}

SysArips& SysArips::operator <<(AripsCmd cmd) {
	SendByte(cmd);
	return *this;
}

void SysArips::SetServos(int mask, float* pos, int len) {
	int corrected[len];
	int index = 0, m = 1;
	for(int i = 0; (i < 8) && (index < len); i++, m <<= 1)
		if(mask & m) {
			float c = Limit(pos[index], 0.0f, 255.0f);
			corrected[index] = c;
			currentServoPos[i] = corrected[index];
			index++;
		}
	
	if(!simulMode) {
		if(mask) {
			*this << ARIPS_SET_SERVO;
			*this << mask;
			
			for(int i=0; i < len; i++)
				*this << CMD_SPACE << corrected[i];

			*this << CMD_END;
		}
	}
	
	for (std::list < boost::function<void()> >::iterator iter = onServoChanged.begin(); iter != onServoChanged.end(); ++iter)	
        (*iter)();
}

void SysArips::DriveAt(float left, float right) {
	// cout << "SysArips::DriveAt " << left << " " << right << endl;
	
	currentMotorLeft = left = Limit(left, -1.0f, 1.0f);
	currentMotorRight = right = Limit(right, -1.0f, 1.0f);
	
    int l = left * 127 + 128, r = right * 127 + 128;
    
	
	//cout << "set speed " << l << " " << r << endl;
	
	if(!simulMode)
	// cout << "raw speed " << currentMotorLeftRaw << " " << currentMotorRightRaw << endl;
		*this << ARIPS_SET_SPEED << l << CMD_SPACE << r << CMD_END;
}

void SysArips::ResetEncoders() {
	if(!simulMode) 
		*this << ARIPS_RESET_ENCODER << CMD_END;
	
	currentDrivenDistLeft = 0;
	currentDrivenDistRight = 0;
	currentDriveDist = 0;
	
	currentAngle = 0;
	currentPosX = currentPosY = 0;
}

void SysArips::ResetServo() {
	if(!simulMode) 
		*this << ARIPS_SET_SERVO << CMD_END;
	
	for(int i=0; i < 8; i++)
		currentServoPos[i] = 127;
}


void SysArips::RefreshAdc() {
	if(simulMode) {
		// leave adc values as they are for now
        for (std::list < boost::function<void()> >::iterator iter = onAdcData.begin(); iter != onAdcData.end(); ++iter)	
            (*iter)();

	} else {
	// cout << "refresh" << endl;
		*this << ARIPS_GET_ADC << CMD_END;
	}
}

void SysArips::UpdatePosition() {
	if(simulMode) {
		long long ticks = GetUsTicks();
		long long tDiff = ticks - oldTime;
		
		// raw motor speed of 100 means 0.1 m/s
		int countL = currentMotorLeft / ENCODER_FACTOR * (double)tDiff / 1000000.0;
		int countR = currentMotorRight / ENCODER_FACTOR * (double)tDiff / 1000000.0;
		
		// fake encoder
		ProcessMotion(countL, countR);
	} else {
		*this << ARIPS_GET_ENCODER << CMD_END;
	}
}

void SysArips::ResetDrivenDistanceAndPos() {
	currentDrivenDistLeft = currentDrivenDistRight = 0;
	currentAngle.value = 0;
	currentPosX = currentPosY = 0;
    
    // TODO
}

void SysArips::AdcPower(bool on) {
	*this << (on ? ARIPS_POWER_ON : ARIPS_POWER_OFF) << ARIPS_POWERMASK_ADC << CMD_END;
}

void SysArips::ServoPower(bool on) {
	*this << (on ? ARIPS_POWER_ON : ARIPS_POWER_OFF) << ARIPS_POWERMASK_SERVO << CMD_END;
}

int SysArips::GetAdc(int channel) const {
	channel = Limit(channel, 0, 7);
	return currentADC[channel];
}

const short* SysArips::GetAdcValues() const {
	return currentADC;
}

void SysArips::AddOnAdc(const boost::function<void()>& f) {
	onAdcData.push_back(f);
}

void SysArips::AddOnNewPostition(const boost::function<void()>& f) {
	onNewPosition.push_back(f);
}

void SysArips::AddOnServoChanged(const boost::function<void()>& f) {
	onServoChanged.push_back(f);
}

void SysArips::DoCommand() {
	int size = aripsParams.size();
	

	/* if (aripsCommand == MINI_SERVO) {
		if (size > 1) {
			int off = aripsParams.front() & 0x7;
			aripsParams.pop_front();

			for (int i = 1; i < size; i++) {
				servopos[off] = aripsParams.front();
				aripsParams.pop_front();
				off = (off + 1) & 0x7;
			}

			long long timestamp = GetSystemMillis();

			for (list<Client*>::iterator iter = Client::allClients.begin(); iter != Client::allClients.end();) {
				Client *c = *iter++;

				try {
					c->SendServosData(timestamp);
				} catch (const IOException& ex) {
					cout << "Caught exception: " << ex.what() << endl;
					delete c;
				}
			}
		}
	} else*/ if (aripsCommand == MINI_ADC) {
		if (size > 1) {
			int off = aripsParams.front() & 0x7;
			aripsParams.pop_front();

			for (int i = 1; i < size; i++) {
				int val = aripsParams.front();
				currentADC[off] = val;
				aripsParams.pop_front();
				off = (off + 1) & 0x7;
			}

			//for(list<Client*>::iterator iter = Client::allClients.begin(); iter != Client::allClients.end(); ++iter)
			//	(*iter)->AdcReady();
			// InformClients(&Client::AdcReady);
			for (std::list < boost::function<void()> >::iterator iter = onAdcData.begin(); iter != onAdcData.end(); ++iter)	
                (*iter)();
		}
	}/** else if(aripsCommand == MINI_POSITION)
	{
		if(size == 3)
		{
			posX = aripsParams.front();
			aripsParams.pop_front();
			posY = aripsParams.front();
			aripsParams.pop_front();
			angle = aripsParams.front() % 360;
			aripsParams.pop_front();

			for(list<Client*>::iterator iter = Client::allClients.begin(); iter != Client::allClients.end(); ++iter)
				(*iter)->PosReady();
		}
	} 
	else if (aripsCommand == MINI_ENCODER) {
		if (size == 2) {
			countL = aripsParams.front();
			aripsParams.pop_front();
			countR = aripsParams.front();
			aripsParams.pop_front();
		}
	} */ else if (aripsCommand == MINI_ENCODER) {
		if (size == 2) {
			int dCountL = aripsParams.front();
			aripsParams.pop_front();
			int dCountR = aripsParams.front();
			aripsParams.pop_front();

			// if(dCountL | dCountR)
			// cout << "MINI_ENCODER " << dCountL << " " << dCountR << endl;

			ProcessMotion(dCountL, dCountR);
		}
	}
}
