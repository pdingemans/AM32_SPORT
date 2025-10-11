
#include "ADC.h"
#include "at32f421_it.h"
#include "functions.h"
#include "targets.h"

#ifdef USE_ADC

#ifdef PA6_NTC_ONLY
uint16_t ADCDataDMA[1];
#else
#ifdef USE_ADC_INPUT
uint16_t ADCDataDMA[4];
#else
uint16_t ADCDataDMA[4];
#endif
#endif

extern uint16_t ADC_raw_temp;
extern uint16_t ADC_raw_volts;
extern uint16_t ADC_raw_current;
extern uint16_t ADC_raw_input;

#ifdef USE_NTC
int NTC_table[65] = {
    400, 332, 264, 230, 208, 192, 180, 170, 161,
    154, 147, 141, 136, 131, 127, 122, 119, 115,
    111, 108, 105, 102, 99, 96, 94, 91, 88, 86,
    84, 81, 79, 77, 74, 72, 70, 68, 66, 63, 61,
    59, 57, 55, 53, 50, 48, 46, 44, 41, 39, 37,
    34, 32, 29, 26, 23, 20, 16, 13, 9, 4, -1,
    -8, -16, -29, -42};
#endif

void ADC_DMA_Callback()
{ // read dma buffer and set extern variables
#ifdef PA6_NTC_ONLY
    ADC_raw_temp = ADCDataDMA[0];
#else
#ifdef USE_ADC_INPUT
    ADC_raw_temp = ADCDataDMA[3];
    ADC_raw_volts = ADCDataDMA[1] / 2;
    ADC_raw_current = ADCDataDMA[2];
    ADC_raw_input = ADCDataDMA[0];
#else
    ADC_raw_temp = ADCDataDMA[3];
#ifdef PA6_VOLTAGE
    ADC_raw_volts = ADCDataDMA[1];
    ADC_raw_current = ADCDataDMA[0];
#else
    ADC_raw_volts = ADCDataDMA[0];
    ADC_raw_current = ADCDataDMA[1];
#endif
#endif
#endif
}

void ADC_Init(void)
{
#ifdef PA2_VOLTAGE
    gpio_mode_QUICK(GPIOA, GPIO_MODE_ANALOG, GPIO_PULL_NONE, GPIO_PINS_2);
#else
    gpio_mode_QUICK(GPIOA, GPIO_MODE_ANALOG, GPIO_PULL_NONE, CURRENT_ADC_PIN);
#endif
    gpio_mode_QUICK(GPIOA, GPIO_MODE_ANALOG, GPIO_PULL_NONE, VOLTAGE_ADC_PIN);
#ifdef USE_NTC
    gpio_mode_QUICK(GPIOA, GPIO_MODE_ANALOG, GPIO_PULL_NONE, TEMP_ADC_PIN);
#endif

    dma_init_type dma_init_struct;
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    // nvic_irq_enable(DMA1_Channel1_IRQn, 6, 0);
    dma_reset(DMA1_CHANNEL1);
    dma_default_para_init(&dma_init_struct);
#ifdef PA6_NTC_ONLY
    dma_init_struct.buffer_size = 1;
#else
    dma_init_struct.buffer_size = 4;
#endif
    dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_base_addr = (uint32_t)&ADCDataDMA;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_base_addr = (uint32_t)&ADC1->odt;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma_init_struct.peripheral_inc_enable = FALSE;
    // dma_init_struct.priority = DMA_PRIORITY_HIGH;
    dma_init_struct.priority = DMA_PRIORITY_LOW;

    dma_init_struct.loop_mode_enable = TRUE;

    dma_init(DMA1_CHANNEL1, &dma_init_struct);

    // Enable DMA error interrupt
    // dma_interrupt_enable(DMA1_CHANNEL1, DMA_DTERR_INT, TRUE);
    // dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);

    adc_base_config_type adc_base_struct;
    crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
    crm_adc_clock_div_set(CRM_ADC_DIV_16);

    adc_base_default_para_init(&adc_base_struct);
    adc_base_struct.sequence_mode = TRUE;
    adc_base_struct.repeat_mode = TRUE;
    adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
#ifdef PA6_NTC_ONLY
    adc_base_struct.ordinary_channel_length = 1;
    adc_base_config(ADC1, &adc_base_struct);
    adc_ordinary_channel_set(ADC1, ADC_CHANNEL_6, 1, ADC_SAMPLETIME_28_5);
#else
    adc_base_struct.ordinary_channel_length = 4;
    adc_base_config(ADC1, &adc_base_struct);

    adc_tempersensor_vintrv_enable(TRUE);

    adc_ordinary_channel_set(ADC1, VOLTAGE_ADC_CHANNEL, 1, ADC_SAMPLETIME_28_5);
    adc_ordinary_channel_set(ADC1, CURRENT_ADC_CHANNEL, 2, ADC_SAMPLETIME_28_5);
    adc_ordinary_channel_set(ADC1, ADC_CHANNEL_17, 3, ADC_SAMPLETIME_239_5); // internal reference voltage
    adc_ordinary_channel_set(ADC1, TEMP_ADC_CHANNEL, 4, ADC_SAMPLETIME_239_5);
#endif

    adc_enable(ADC1, TRUE);
    adc_calibration_init(ADC1);
    while (adc_calibration_init_status_get(ADC1))
        ;
    adc_calibration_start(ADC1);
    while (adc_calibration_status_get(ADC1))
        ;
    // dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);
    dma_init(DMA1_CHANNEL1, &dma_init_struct);
    dma_channel_enable(DMA1_CHANNEL1, TRUE);
    adc_dma_mode_enable(ADC1, TRUE);
    adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_SOFTWARE, TRUE);
    adc_ordinary_software_trigger_enable(ADC1, TRUE);
}
inline void disable_ADC(void)
{
    dma_channel_enable(DMA1_CHANNEL1, FALSE);
    adc_dma_mode_enable(ADC1, FALSE);
    adc_enable(ADC1, FALSE);
}
inline void enable_ADC(void)
{
    // we need to reset the DMA channel because we stopped the ADC and DMA together somewhere in the loop
    //  restarting the ADC will make it start from 0, so our DMA channel should also start from 0 (in this case 4 elements to transfer)
    dma_data_number_set(DMA1_CHANNEL1, 4); // Set transfer count to 4 (or 1 for PA6_NTC_ONLY)
    adc_dma_mode_enable(ADC1, TRUE);
    dma_channel_enable(DMA1_CHANNEL1, TRUE);
    adc_enable(ADC1, TRUE);
    // everything is enable now, we just need to start the conversion
    adc_ordinary_software_trigger_enable(ADC1, TRUE); // start it again because it doesnt start automatically
}
  
int16_t getConvertedDegrees(uint16_t adcrawtemp)
{

  static int32_t filtered_temp = 0;
    static uint32_t last_millis = 0;


    
#ifdef USE_NTC
    int p1, p2;
    p1 = NTC_table[(adcrawtemp >> 6)];
    p2 = NTC_table[(adcrawtemp >> 6) + 1];
   return p1 - ((p1 - p2) * (adcrawtemp & 0x003F)) / 64;
#else
    // f421 datasheet : {(V25 – VTS) / Avg_Slope} + 25
    // - 40 = 1V , 100 = 1.6 -> mV/C = 100--40 = 140 degrees per 0.6V = 0.6/140 = 0.004285714V = 4.285714mV/C
    // lets do a facto 100 so:
    //
    if (millis() - last_millis >= 1)
    {

        last_millis = millis();
        int64_t temp =  ((int64_t)12800 - (((int64_t)adcrawtemp * (int64_t)33000) / (int64_t)4096)) / (int64_t)-42 + 25;
        temp <<=7; // we do everything in factor 128 to have more precision for the filter
        filtered_temp = filtered_temp  +((temp-filtered_temp)>>7);   // very simple exponential low pass filter

    }
    return filtered_temp>>7; // divide by 128 to get back to normal value
#endif
}
#endif // USE_ADC
