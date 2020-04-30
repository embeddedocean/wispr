/*
 * Spectrum header file
 * ------
 * THIS SOFTWARE IS PROVIDED BY EOS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL EOS OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Embedded Ocean Systems (EOS), 2020
 *
 */
#ifndef _SPECTRUM_H
#define _SPECTRUM_H

#include "arm_math.h"
#include "arm_const_structs.h"

#include "wispr.h"

#define HAMMING_WINDOW 1
#define HANN_WINDOW 2
#define BLACKMAN_WINDOW 3
#define RECT_WINDOW 4

extern void spectrum_update_header(wispr_data_header_t *psd, wispr_data_header_t *adc);

extern int spectrum_init_f32(uint16_t *nbins, uint16_t nfft, uint16_t overlap, uint8_t wintype);
extern int spectrum_f32(wispr_data_header_t *psd, float32_t *psd_data, wispr_data_header_t *adc, uint8_t *adc_data, uint16_t nsamps);

extern int spectrum_init_q31(uint16_t *nbins, uint16_t nfft, uint16_t overlap, uint8_t wintype);
extern int spectrum_q31(wispr_data_header_t *psd, float32_t *psd_data, wispr_data_header_t *adc, uint8_t *adc_data, uint16_t nsamps);

extern void spectrum_window(float32_t *w, uint8_t type, uint16_t size);

//extern int spectrum_q31(q31_t *input, float32_t *output, uint16_t nsamps, uint16_t nbins, uint16_t nfft, uint16_t overlap );
//extern int init_spectrum_q31(uint16_t nfft, uint8_t win_type );


#endif /* _SPECTRUM_H */

