#include "OnlineGRFSphereHalfSpaceForce.h"

#include <OpenSim/Common/Object.h>
#include <OpenSim/OpenSim.h>

#ifdef _WIN32
#include <windows.h>

BOOL APIENTRY DllMain(HMODULE, DWORD reason, LPVOID) {
    if (reason == DLL_PROCESS_ATTACH) {
        OpenSim::Object::registerType(OnlineGRFSphereHalfSpaceForce());
    }
    return TRUE;
}
#else
__attribute__((constructor))
void registerOnlineGRFContact() {
    OpenSim::Object::registerType(OnlineGRFSphereHalfSpaceForce());
}
#endif
