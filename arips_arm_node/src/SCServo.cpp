/*
 * SCServo.cpp
 * Ӳ��ͨ�Žӿ�
 * ����: 2016.8.9
 * ����: ̷����
 */


#include "SCServo.h"

#include <QDebug>

SCServo::SCServo()
{
	IOTimeOut = 2;
    pSerial.setPortName("/dev/ttyUSB0");
    pSerial.setBaudRate(1000000);
    pSerial.open(QIODevice::ReadWrite);
    qDebug() << "isOpen: " << pSerial.isOpen();
}

int SCServo::readSCS(unsigned char *nDat, int nLen)
{
    int readSize = 0;

    readSize += pSerial.read((char*)nDat + readSize, nLen - readSize);

    while (readSize < nLen) {
        if(!pSerial.waitForReadyRead(1))
            return 0;
        readSize += pSerial.read((char*)nDat + readSize, nLen - readSize);
    }

    return readSize;
}

int SCServo::writeSCS(unsigned char *nDat, int nLen)
{
    int n = pSerial.write((char*)nDat, nLen);
    pSerial.waitForBytesWritten(1000);
    return n;
}

int SCServo::writeSCS(unsigned char bDat)
{
    int n = pSerial.write((char*)&bDat, 1);
    pSerial.waitForBytesWritten(1000);
    return n;
}

void SCServo::flushSCS()
{
    pSerial.readAll();
}
