#ifndef MOV_MED_FILT
#define MOV_MED_FILT



#include "stdint.h"

typedef struct movingMedian_s
{
  float *buffer;      /**< Data buffer pointer */
  uint16_t size;        /**< Size of filter buffer */
  uint16_t index;       /**< Current location in buffer */
  float fill;        /**< Buffer filled level */
  float filtered;     /**< Filtered output */
  uint16_t sample_time; /**< data sampling time interval */
  uint32_t last_time;   /**< last sampled time */
} movingMedian_t;


void moving_median_create(movingMedian_t *context, uint16_t filter_size, uint16_t sample_time);


void moving_median_filter(movingMedian_t *context, float input);

#endif // MOV_MED_FILT
