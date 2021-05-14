#ifndef MAIN_H
#define MAIN_H

#include "camera/dcmi_camera.h"
#include <msgbus/messagebus.h>
#include <parameter/parameter.h>

#define PI 3.141565

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#endif /* PROJECT_MAIN_H_ */
