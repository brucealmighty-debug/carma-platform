/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class gov_dot_fhwa_saxton_carma_message_factory_MobilityAckMessage */

#ifndef _Included_gov_dot_fhwa_saxton_carma_message_factory_MobilityAckMessage
#define _Included_gov_dot_fhwa_saxton_carma_message_factory_MobilityAckMessage
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     gov_dot_fhwa_saxton_carma_message_factory_MobilityAckMessage
 * Method:    encode_MobilityAck
 * Signature: ([I[I[I[II[B)[B
 */
JNIEXPORT jbyteArray JNICALL Java_gov_dot_fhwa_saxton_carma_message_factory_MobilityAckMessage_encode_1MobilityAck
  (JNIEnv *, jobject, jintArray, jintArray, jintArray, jintArray, jint, jbyteArray);

/*
 * Class:     gov_dot_fhwa_saxton_carma_message_factory_MobilityAckMessage
 * Method:    decode_MobilityAck
 * Signature: ([B[B[B[B[ILjava/lang/Object;[B)I
 */
JNIEXPORT jint JNICALL Java_gov_dot_fhwa_saxton_carma_message_factory_MobilityAckMessage_decode_1MobilityAck
  (JNIEnv *, jobject, jbyteArray, jbyteArray, jbyteArray, jbyteArray, jintArray, jobject, jbyteArray);

#ifdef __cplusplus
}
#endif
#endif