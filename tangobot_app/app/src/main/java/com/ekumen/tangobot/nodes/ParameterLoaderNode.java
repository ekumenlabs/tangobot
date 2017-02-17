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

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.logging.Log;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.yaml.snakeyaml.Yaml;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * Loads parameters from Yaml files into ROS Parameter Server.
 * A different namespace can be specified for each of the files to load.
 * To use this node, create a NamedResource array with the resources to load,
 * and start it in the standard RosJava way.
 *
 * @author lucas@ekumenlabs.com (Lucas Chiesa).
 * Modified by jubeira@ekumenlabs.com (Juan I. Ubeira)
 */
public class ParameterLoaderNode extends AbstractNodeMain {

    public static final String NODE_NAME = "parameter_loader";
    private final List<Pair<String, LoadedRawResource>> params = new ArrayList<>();
    private Log log;

    /**
     * Default constructor
     * @param resources Array of resources with their respective namespace to load.
     */
    public ParameterLoaderNode(ArrayList<Pair<InputStream, String>> resources) {
        for(Pair<InputStream, String> nr : resources) {
            addSingleYmlInput(nr.getLeft(), nr.getRight() == null ? "" : nr.getRight());
        }
    }

    private void addSingleYmlInput(InputStream ymlInputStream, String namespace) {
        this.params.add(new ImmutablePair<>(namespace, new LoadedRawResource((new Yaml()).load(ymlInputStream))));
    }

    private void addParams(ParameterTree parameterTree, String namespace, Map<String, Object> params) {
        for(Map.Entry<String, Object> e: params.entrySet()) {
            String fullKeyName = namespace + "/" + e.getKey();
            if (log != null) {
                log.info("Loading parameter " + fullKeyName + " \nValue = " + e.getValue());
            }

            if(e.getValue() instanceof String) {
                parameterTree.set(fullKeyName, (String)e.getValue());
            } else if(e.getValue() instanceof Integer) {
                parameterTree.set(fullKeyName, (Integer)e.getValue());
            } else if(e.getValue() instanceof Double) {
                parameterTree.set(fullKeyName, (Double)e.getValue());
            } else if(e.getValue() instanceof Map) {
                parameterTree.set(fullKeyName, (Map)e.getValue());
            } else if(e.getValue() instanceof Boolean) {
                parameterTree.set(fullKeyName, (Boolean)e.getValue());
            } else if(e.getValue() instanceof List) {
                parameterTree.set(fullKeyName, (List)e.getValue());
            } else if (log != null) {
                log.debug("I don't know what type parameter " + fullKeyName + " is. Value = " + e.getValue());
                log.debug("Class name is: " + e.getValue().getClass().getName());
            }
        }
    }

    /**
     * @return the name of the Node that will be used if a name was not
     * specified in the Node's associated NodeConfiguration
     */
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(NODE_NAME);
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        if(params != null) {
            ParameterTree parameterTree = connectedNode.getParameterTree();
            log = connectedNode.getLog();

            // TODO: For some reason, setting the / param when using a rosjava master doesn't work
            // It does work fine with an external master, and also setting other params of any type
            for (Pair<String, LoadedRawResource> m : params) {
                addParams(parameterTree, m.getLeft(), m.getRight().resource);
            }

            connectedNode.shutdown();
        }
    }

    /**
     * Thin wrapper for the object returned by Yaml.load().
     * The object returned by Yaml.load() is a LinkedHashMap<String, Object>; this class is to
     * keep the code simple.
     */
    private class LoadedRawResource {
        public Map<String, Object> resource;

        LoadedRawResource(Object resource) {
            this.resource = (Map<String, Object>) resource;
        }
    }
}
