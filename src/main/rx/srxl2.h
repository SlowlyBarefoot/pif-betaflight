#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "pg/rx.h"

#include "rx/rx.h"

struct StPifStreamBuffer;

bool srxl2RxInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState);
bool srxl2RxIsActive(void);
void srxl2RxWriteData(const void *data, int len);
bool srxl2TelemetryRequested(void);
void srxl2InitializeFrame(struct StPifStreamBuffer *dst);
void srxl2FinalizeFrame(struct StPifStreamBuffer *dst);
void srxl2Bind(void);
