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
import org.ros.node.Node;

import java.util.ArrayList;

// Note: this extension can be incorporated to the rosjava_core ParameterLoaderNode;
// in that case this file shall be removed from this project.

public class ParameterLoaderNode extends org.ros.helpers.ParameterLoaderNode {
    private UserHook userHook;
    private Object onSuccessPayload;

    /**
     * Default constructor
     * @param resources Array of resources with their respective namespace to load.
     */
    public ParameterLoaderNode(ArrayList<Resource> resources) {
        this(resources, null, null);
    }

    /**
     * User-hook constructor
     * Sets user callbacks to perform actions on error or on success.
     * @param resources Array of resources with their respective namespace to load.
     * @param hook Object that defines onError and onSuccess callbacks.
     * @param onSuccessPayload Object to send as a parameter to onSuccess callback defined by hook.
     */
    public ParameterLoaderNode(ArrayList<Resource> resources, UserHook hook, Object onSuccessPayload) {
        super(resources);
        // Set dummy hook if received a null one
        if (hook == null) {
            userHook = new UserHook() {
                @Override
                public void onSuccess(Object o) {}
                @Override
                public void onError(Throwable t) {}
            };
        } else {
            userHook = hook;
        }

        this.onSuccessPayload = onSuccessPayload;
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        try {
            super.onStart(connectedNode);
        } catch (Exception e) {
            userHook.onError(e);
        }
    }

    @Override
    public void onShutdownComplete(Node node) {
        super.onShutdownComplete(node);
        userHook.onSuccess(onSuccessPayload);
    }

    @Override
    public void onError(Node node, Throwable throwable) {
        super.onError(node, throwable);
        userHook.onError(throwable);
    }

    public interface UserHook {
        void onSuccess(Object o);
        void onError(Throwable t);
    }
}
