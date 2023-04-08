//
// Created by waxz on 23-3-31.
//

#ifndef JNI_HPP_STRING_UTIL_H
#define JNI_HPP_STRING_UTIL_H
#include <iostream>
#include <string>
#include <locale>
#include <codecvt>
#include <memory>

namespace common{


    template <typename T>
    std::string toUTF8(const std::basic_string<T, std::char_traits<T>, std::allocator<T>>& source)
    {
        std::string result;

        std::wstring_convert<std::codecvt_utf8_utf16<T>, T> convertor;
        result = convertor.to_bytes(source);

        return result;
    }

    template <typename T>
    void fromUTF8(const std::string& source, std::basic_string<T, std::char_traits<T>, std::allocator<T>>& result)
    {
        std::wstring_convert<std::codecvt_utf8_utf16<T>, T> convertor;
        result = convertor.from_bytes(source);
    }


}

#endif //JNI_HPP_STRING_UTIL_H
