/*
 * Copyright 2017 Ekumen, Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.ekumen.tangobot.nodes;

import org.ros.node.ConnectedNode;

import java.util.ArrayList;
import java.util.concurrent.CountDownLatch;

// Note: this extension can be incorporated to the rosjava_core ParameterLoaderNode;
// in that case this file shall be removed from this project.

public class ParameterLoaderNode extends org.ros.helpers.ParameterLoaderNode {
    private CountDownLatch latch;

    /**
     * Latched constructor
     * @param resources Array of resources with their respective namespace to load.
     * @param latch Latch to decrement after loading all parameters to sync with other threads.
     */
    public ParameterLoaderNode(ArrayList<Resource> resources, CountDownLatch latch) {
        super(resources);
        this.latch = latch;
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        super.onStart(connectedNode);
        if (latch != null) {
            latch.countDown();
        }
    }
}
