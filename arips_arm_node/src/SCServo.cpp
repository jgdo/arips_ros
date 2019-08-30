/*
 * SCServo.cpp
 * Ӳ��ͨ�Žӿ�
 * ����: 2016.8.9
 * ����: ̷����
 */


#include "SCServo.h"

#include <QDebug>
#include <stdexcept>

SCServo::SCServo(int baud)
{
	IOTimeOut = 2;
    pSerial.setPortName("/dev/ttyUSB1");
    pSerial.setBaudRate(baud);
    pSerial.open(QIODevice::ReadWrite);
    qDebug() << "isOpen: " << pSerial.isOpen();
}

void SCServo::readSCS(unsigned char *nDat, int nLen)
{
    int readSize = 0;

    readSize += pSerial.read((char*)nDat + readSize, nLen - readSize);

    while (readSize < nLen) {
        if(!pSerial.waitForReadyRead(20))
            throw std::runtime_error("Error reading serial port: timeout.");
        readSize += pSerial.read((char*)nDat + readSize, nLen - readSize);
    }
}

int SCServo::writeSCS(unsigned char *nDat, int nLen)
{
    int n = pSerial.write((char*)nDat, nLen);
    pSerial.waitForBytesWritten(-1);
    return n;
}

int SCServo::writeSCS(unsigned char bDat)
{
    int n = pSerial.write((char*)&bDat, 1);
    pSerial.waitForBytesWritten(-1);
    return n;
}

void SCServo::flushSCS()
{
    pSerial.readAll();
}

void SCServo::setBaud(int baud) {
    pSerial.setBaudRate(baud);
}
