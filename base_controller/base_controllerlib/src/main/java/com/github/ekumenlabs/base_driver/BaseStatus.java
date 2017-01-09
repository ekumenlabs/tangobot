/*
 * Copyright (C) 2013 Creativa77.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package com.github.ekumenlabs.base_driver;

/**
 * Created by Lucas Chiesa on 10/10/13.
 */

public class BaseStatus {

    // Robot state variables
    private short timeStamp = 0;
	private byte bumper = 0;
	private byte wheelDrop = 0;
	private byte cliff = 0;
	private byte button = 0;
	private byte charger = 0;
	private byte battery = 0;
	private short angle = 0;
	private short angleRate = 0;
	private int leftDistance = 0;
	private int rightDistance = 0;

    public short getTimestamp() {
        return timeStamp;
    }

    public void setTimeStamp(short timeStamp) {
        this.timeStamp = timeStamp;
    }

    public byte getWheelDrop() {
        return wheelDrop;
    }

    public void setWheelDrop(byte wheelDrop) {
        this.wheelDrop = wheelDrop;
    }

    public byte getCliff() {
        return cliff;
    }

    public void setCliff(byte cliff) {
        this.cliff = cliff;
    }

    public byte getButton() {
        return button;
    }

    public void setButton(byte button) {
        this.button = button;
    }

    public byte getCharger() {
        return charger;
    }

    public void setCharger(byte charger) {
        this.charger = charger;
    }

    public byte getBattery() {
        return battery;
    }

    public void setBattery(byte battery) {
        this.battery = battery;
    }

    public short getAngle() {
        return angle;
    }

    public void setAngle(short angle) {
        this.angle = angle;
    }

    public short getAngleRate() {
        return angleRate;
    }

    public void setAngleRate(short angleRate) {
        this.angleRate = angleRate;
    }

    public int getLeftDistance() {
        return leftDistance;
    }

    public void setLeftDistance(int leftDistance) {
        this.leftDistance = leftDistance;
    }

    public int getRightDistance() {
        return rightDistance;
    }

    public void setRightDistance(int rightDistance) {
        this.rightDistance = rightDistance;
    }

    public byte getBumper() {
        return bumper;
    }

    public void setBumper(byte bumper) {
        this.bumper = bumper;
    }
}