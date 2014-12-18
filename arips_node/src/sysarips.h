#ifndef SYSARIPS_H_INCLUDED
#define SYSARIPS_H_INCLUDED

#include <stdint.h>
#include <time.h>
#include <list>
#include <map>
#include <list>

#include <boost/function.hpp>

#include "angle.h"

#include "commands.h"


// TODO get from parameter server and adjust for ARIPS 7
#define WHEEL_DIST 0.335 // meter

#define ADC_CHANNEL_VOLTAGE 7
#define ADC_VOLTAGE_FACTOR 0.058894

#define ENCODER_FACTOR 0.000863937

class SysArips {
public:
	SysArips();

	void StartSimulation();
	int Open();
	void Close();
	void ReadInput();

	void SetServos(int mask, float* pos, int len);
	void DriveAt(float left, float right);

	void ResetEncoders();

	void RefreshServo();
	void RefreshAdc();

	void UpdatePosition();

	void ResetServo();
	void ResetDrivenDistanceAndPos();

	void AdcPower(bool on);
	void ServoPower(bool on);

	int GetAdc(int channel) const;
	const short* GetAdcValues() const;

	void AddOnAdc(const boost::function<void()>& f);
	void AddOnNewPostition(const boost::function<void()>& f);
	void AddOnServoChanged(const boost::function<void()>& f);

	inline double GetDrivenDistLeft() const {
		return currentDrivenDistLeft * ENCODER_FACTOR;
	}

	inline double GetDrivenDistRight() const {
		return currentDrivenDistRight * ENCODER_FACTOR;
	}

	inline double GetDriveDist() const {
		return currentDriveDist * ENCODER_FACTOR;
	}

	inline float GetSpeed() const {
		return currentSpeed;
	}

	inline float GetAngleSpeed() const {
		return currentAngleSpeed;
	}

	inline float GetMotorLeft() const {
		return currentMotorLeft;
	}

	inline float GetMotorRight() const {
		return currentMotorRight;
	}

	inline const float* GetServos() const {
		return currentServoPos;
	}

	inline int GetUartFD() const {
		return uartfd;
	}

	inline double getPosX() const {
		return currentPosX;
	}

	inline double getPosY() const {
		return currentPosY;
	}

	inline Radian<double> getAngle() const {
		return currentAngle;
	}

	inline float GetWheelDist() const {
		return wheelDist;
	}

private:
	bool simulMode; // simulator mode

	float wheelDist;

	int uartfd;
	long long oldTime;

	int aripsCommand;
	std::list<int> aripsParams;

	short currentADC[8];

	long long currentDrivenDistLeft, currentDrivenDistRight, currentDriveDist;

	Radian<double> currentAngle;
	double currentPosX, currentPosY;
	float currentSpeed, currentAngleSpeed;
	float currentMotorRight, currentMotorLeft;

	float currentServoPos[8];

	std::list<boost::function<void()> > onAdcData;
	std::list<boost::function<void()> > onNewPosition;
	std::list<boost::function<void()> > onServoChanged;

	void SendByte(int b);

	void ProcessByte(int byte);

	SysArips& operator <<(uint8_t value);
	SysArips& operator <<(int8_t value);
	SysArips& operator <<(uint16_t value);
	SysArips& operator <<(int16_t value);
	SysArips& operator <<(uint32_t value);
	SysArips& operator <<(int32_t value);
	SysArips& operator <<(CommonCmd cmd);
	SysArips& operator <<(AripsCmd cmd);

	void ProcessMotion(int dCountL, int dCountR);

	void DoCommand();
};

extern SysArips arips;

#endif // SYSARIPS_H_INCLUDED
