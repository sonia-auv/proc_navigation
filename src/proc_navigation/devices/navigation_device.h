/**
 * \file	navigation_device.h
 * \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	9/14/2016
 *
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PROC_NAVIGATION_NAVIGATIONDEVICE_H
#define PROC_NAVIGATION_NAVIGATIONDEVICE_H

#include <mutex>
namespace proc_navigation {

    class NavigationDevice {

    public:
        NavigationDevice() : new_data_ready_(false) { };
        void SetNewDataReady();
        bool IsNewDataReady();
        void SetNewDataUsed();

    private:
        bool new_data_ready_;
        std::mutex data_ready_mutex_;
    };

    inline void NavigationDevice::SetNewDataReady() {
        std::lock_guard<std::mutex> guard(data_ready_mutex_);
        new_data_ready_ = true;
    };

    inline bool NavigationDevice::IsNewDataReady() {
        std::lock_guard<std::mutex> guard(data_ready_mutex_);
        return new_data_ready_;
    };

    inline void NavigationDevice::SetNewDataUsed() {
        std::lock_guard<std::mutex> guard(data_ready_mutex_);
        new_data_ready_ = false;
    };

}
#endif //PROC_NAVIGATION_NAVIGATIONDEVICE_H
