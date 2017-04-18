// Copyright 2017 Ekumen, Inc. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <jni.h>
/* Header for class com_ekumen_tangobot_application_MoveBaseNode */

#ifndef _Included_com_ekumen_tangobot_application_MoveBaseNode
#define _Included_com_ekumen_tangobot_application_MoveBaseNode
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     com_ekumen_tangobot_application_MoveBaseNode
 * Method:    execute
 * Signature: (Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;[Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_com_ekumen_tangobot_application_MoveBaseNode_execute
  (JNIEnv *, jobject, jstring, jstring, jstring, jobjectArray);

/*
 * Class:     com_ekumen_tangobot_application_MoveBaseNode
 * Method:    shutdown
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_com_ekumen_tangobot_application_MoveBaseNode_shutdown
  (JNIEnv *, jobject);

#ifdef __cplusplus
}
#endif
#endif
