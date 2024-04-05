#include "moving-median.h"
#include <stdlib.h>
#include "stm32f4xx_hal.h"

void swap(float * a, float* b)
{
  float temp;

  temp = *a;
  *a = *b;
  *b = temp;
}

void moving_median_create(movingMedian_t *context, int16_t filter_size, uint16_t sample_time)
{

  free(context->buffer);

  if (filter_size % 2 == 0)
  {
    context->size = filter_size - 1;
  }
  else
  {
    context->size = filter_size;
  }

  context->buffer = (float*)malloc(filter_size * sizeof(float));
  context->index = 0;
  context->fill = 0;
  context->filtered = 0;
  context->sample_time = sample_time;
  context->last_time = 0;
}

void moving_median_filter(movingMedian_t *context, float filter_input)
{
  if ((HAL_GetTick() - context->last_time) > context->sample_time)
  {
    context->last_time = HAL_GetTick();
    /* buffer filling */
    if (context->fill < context->size)
    {
      context->fill++;
    }
    float *sort_buffer = (float*)malloc(context->fill * sizeof(float));
    /* Replace old data by new */
    context->buffer[context->index] = filter_input;

    /* Copy buffer to temp sort buffer */
    for (uint16_t i = 0; i < context->fill; i++)
    {
      sort_buffer[i] = context->buffer[i];
    }
    /* Sort temporary buffer */
    for (uint16_t i = 0; i < context->fill; i++)
    {
      for (uint16_t j = i + 1; j < context->fill; j++)
      {
        if (sort_buffer[j] < sort_buffer[i])
        {
          swap(&sort_buffer[j], &sort_buffer[i]);
        }
      }
    }
    /* Increment buffer index */
    context->index++;
    if (context->index >= context->size)
    {
      context->index = 0;
    }
    /* sorted buffer med value */
    context->filtered = sort_buffer[(uint16_t)(context->fill / 2)];
    /* Delete temporary sort buffer */
    free(sort_buffer);
  }
}



