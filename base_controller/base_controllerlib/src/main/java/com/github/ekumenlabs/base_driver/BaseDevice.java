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
 * Created by Lucas Chiesa on 30/09/13.
 */

public interface BaseDevice {

    /**
     * initialize the base. This method should is called by the node
     * before sending movement commands.
     */
    void initialize();

    /**
     * moves the base. The argument values are the ones
     * transmitted in a twist message.
     * @param linearVelX: linear speed
     * @param angVelZ: rotational speed
     */
    void move(double linearVelX, double angVelZ);

    /**
     * @return: The base status updated with the latest base information.
     */
    BaseStatus getBaseStatus();


    OdometryStatus getOdometryStatus();
}