#include "LocalSR04DistanceSensor.h"

namespace Local
{
LocalSR04DistanceSensor::LocalSR04DistanceSensor(uint8_t trig, uint8_t echo)
{
    m_trig = trig;
    m_echo = echo;
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
}
uint32_t LocalSR04DistanceSensor::GetDistance()
{
    digitalWrite(m_trig, LOW);
    delayMicroseconds(2);
    digitalWrite(m_trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(m_trig, LOW);
    uint32_t us = pulseIn(m_echo, HIGH);
    return m_lastDistance = us * 0.1715;
}
void LocalSR04DistanceSensor::Debug()
{
    if (reinterpret_cast<ClearRequest*>(m_req)->Clear == 0) return;
    Serial.print("Distance: ");
    Serial.print(m_lastDistance);
    Serial.println("mm");
}
}
