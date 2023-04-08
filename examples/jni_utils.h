//
// Created by waxz on 23-3-31.
//

#ifndef JNI_HPP_JNI_UTILS_H
#define JNI_HPP_JNI_UTILS_H
#include <iostream>

#include <jni/jni.hpp>
#include <jni/types.hpp>

#ifdef _JAVASOFT_JNI_H_
using JNINativeInterface = JNINativeInterface_;
#endif

struct TestEnv : public jni::JNIEnv
{
    TestEnv()
            : jni::JNIEnv { new JNINativeInterface },
              fns(const_cast<JNINativeInterface*>(jni::JNIEnv::functions))
    {
        fns->ExceptionCheck = [] (JNIEnv* env) -> jboolean
        {
            return reinterpret_cast<TestEnv*>(env)->exception ? JNI_TRUE : JNI_FALSE;
        };
    }

    ~TestEnv() { delete fns; }

    bool exception = false;
    JNINativeInterface* fns;
};


template < class T >
struct Testable
{
    T& Ref() { return reinterpret_cast<T&>(*this); }
    T* Ptr() { return reinterpret_cast<T*>( this); }

    bool operator==(const T& other) { return &other == reinterpret_cast<T*>(this); }
};

struct JNIBase
{
    JNIBase(JNIEnv&) {
        std::cout << "Native peer initialized" << std::endl;
    }
    JNIBase(const JNIBase&) = delete; // noncopyable
//    virtual ~JNIBase()= 0;
    virtual void close(JNIEnv&) = 0;
};


#endif //JNI_HPP_JNI_UTILS_H
