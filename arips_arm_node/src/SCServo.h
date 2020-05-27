/*
 * SCServo.h
 * Ӳ��ͨ�Žӿ�
 * ����: 2016.8.25
 * ����: ̷����
 */

#ifndef _SCSERVO_H
#define _SCSERVO_H

#include "SCSProtocol.h"

#include <QSerialPort>
#include <QString>

class SCServo : public SCSProtocol
{
public:
	SCServo(int baud, QString port);
    virtual int writeSCS(unsigned char *nDat, int nLen);
    virtual void readSCS(unsigned char *nDat, int nLen);
    virtual int writeSCS(unsigned char bDat);
    virtual void flushSCS();

	void setBaud(int baud);
public:
        unsigned long int IOTimeOut;
        QSerialPort pSerial;
};

#endif
