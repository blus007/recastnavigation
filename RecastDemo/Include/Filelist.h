//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#ifndef FILELIST_H
#define FILELIST_H

#include <vector>
#include <string>
#include <functional>

void scanDirectoryAppend(const std::string& path, const std::string& ext, std::vector<std::string>& fileList);
void scanDirectory(const std::string& path, const std::string& ext, std::vector<std::string>& fileList);

std::string getFileName(const std::string& filePath);
bool readLine(std::function<int(char*, int)> readFunc, char*& buffer, const int maxSize, int& start, int& size, char*& str, bool& readEnd);

#endif // FILELIST_H
