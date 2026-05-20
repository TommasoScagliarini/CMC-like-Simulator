/* -------------------------------------------------------------------------- *
 * Plugin_interface.cpp                                  *
 * -------------------------------------------------------------------------- */
// Includiamo gli header di TUTTI gli oggetti che vogliamo registrare
#include "SeriesElasticActuator.h"

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/RegisterTypes_osimCommon.h>

// Header standard di OpenSim e C++
#include <OpenSim/OpenSim.h>
#include <cstdio> 

using namespace OpenSim;
using namespace std;

// =========================================================================
// LOGICA DI AUTO-REGISTRAZIONE (ADATTATA PER WINDOWS E MAC/LINUX)
// =========================================================================

#ifdef _WIN32

    // -------------------------------------------------------------------------
    // 1. VERSIONE WINDOWS (DllMain standard)
    // -------------------------------------------------------------------------
    #include <windows.h>

    // Su Windows, questa è la funzione di ingresso standard per le DLL.
    // Viene chiamata automaticamente dal sistema operativo.
    BOOL APIENTRY DllMain(HMODULE hModule,
                          DWORD  ul_reason_for_call,
                          LPVOID lpReserved)
    {
        switch (ul_reason_for_call)
        {
        case DLL_PROCESS_ATTACH:
            // Codice eseguito quando la DLL viene caricata
            printf("\n\n****************************************************************\n");
            printf("[DEBUG WIN] >>> SYSTEM ALERT: IL PLUGIN E' STATO CARICATO IN MEMORIA.\n");
            printf("[DEBUG WIN] >>> INIZIO REGISTRAZIONE OGGETTI...\n");
            
            try {
                Object::registerType(SeriesElasticActuator());
                printf("[DEBUG WIN] >>> SEA: registered.\n");
            } 
            catch (const std::exception& e) {
                printf("[DEBUG WIN] >>> ERROR DURING REGISTRATION: %s \n", e.what());
            }
            printf("****************************************************************\n\n");
            break;

        case DLL_THREAD_ATTACH:
        case DLL_THREAD_DETACH:
        case DLL_PROCESS_DETACH:
            break;
        }
        return TRUE;
    }

#else

    // -------------------------------------------------------------------------
    // 2. VERSIONE ORIGINALE (MAC / LINUX)
    // -------------------------------------------------------------------------

    // Questa funzione è il "Costruttore della Libreria" per GCC/Clang.
    // Il sistema operativo la esegue AUTOMATICAMENTE appena carica il file .dylib/.so.
    __attribute__((constructor))
    void DllMain() {
        
        printf("\n\n****************************************************************\n");
        printf("[DEBUG] >>> SYSTEM ALERT: IL PLUGIN E' STATO CARICATO IN MEMORIA.\n");
        printf("[DEBUG] >>> INIZIO REGISTRAZIONE OGGETTI...\n");
        fflush(stdout);

        try {
            Object::registerType(SeriesElasticActuator());
            printf("[DEBUG] >>> SEA: registered.\n");
            
            printf("****************************************************************\n\n");
            fflush(stdout);
        } 
        catch (const std::exception& e) {
            printf("[DEBUG] >>> ERROR DURING REGISTRATION: %s \n", e.what());
            fflush(stdout);
        }
    }

#endif