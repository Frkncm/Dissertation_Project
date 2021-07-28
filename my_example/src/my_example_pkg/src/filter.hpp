#ifndef _FILTER_HPP
#define _FILTER_HPP

#include <vector>
#include <algorithm>
#include <cmath>

namespace Shimmer3
{

    enum filter_types : int
    {
        LOW_PASS = 0,
        HIGH_PASS,
        BAND_PASS,
        BAND_STOP,
    };

    class Filter
    {

    public:
        Filter()
        {
            filterType = LOW_PASS;

            SetFilterParameters(LOW_PASS, defaultSamplingRate, defaultCornerFrequency, defaultNTaps);
        }

        Filter(int filterType)
        {

            this->filterType = filterType;

            SetFilterParameters(filterType, defaultSamplingRate, defaultCornerFrequency, defaultNTaps);
        }

        Filter(int filterType, double samplingRate, std::vector<double> &&cornerFrequency)
        {

            this->filterType = filterType;

            SetFilterParameters(filterType, samplingRate, cornerFrequency, defaultNTaps);
        }

        Filter(int filterType, double samplingRate, std::vector<double> &&cornerFrequency, int nTaps)
        {

            this->filterType = filterType;

            SetFilterParameters(filterType, samplingRate, cornerFrequency, nTaps);
        }

        void SetFilterParameters(int LoHi, double samplingRate, std::vector<double> &cornerFrequency, int nTaps)
        {

            //reset the buffers
            this->bufferedX.clear();

            if (cornerFrequency.size() != 1)
            {
                if (cornerFrequency[0] > cornerFrequency[1])
                {
                    minCornerFrequency = cornerFrequency[1];
                    maxCornerFrequency = cornerFrequency[0];
                }
                else
                {
                    minCornerFrequency = cornerFrequency[0];
                    maxCornerFrequency = cornerFrequency[1];
                }
            }
            else
                minCornerFrequency = maxCornerFrequency = cornerFrequency[0];

            if (maxCornerFrequency > samplingRate / 2)
            {
                this->validparameters = false;
                std::cout << "Error: cornerFrequency is greater than Nyquist frequency. Please choose valid parameters." << std::endl;
            }
            else
            {
                if (nTaps % 2 != 0)
                {
                    nTaps--;
                    //JOptionPane.showMessageDialog(null, "Warning: nPoles is not an even number. nPoles will be rounded to " +Integer.toString(nTaps));
                }

                if (LoHi == LOW_PASS || LoHi == HIGH_PASS) // High pass or Low pass filter
                {
                    this->samplingRate = samplingRate;
                    this->cornerFrequency = cornerFrequency;
                    this->nTaps = nTaps;

                    double fc = (cornerFrequency[0] / samplingRate);
                    // calculate filter coefficients
                    coefficients = std::vector<double>(nTaps, 0);
                    coefficients = calculateCoefficients(fc, LoHi, nTaps);
                    this->validparameters = true;
                }
                else if (LoHi == BAND_PASS || LoHi == BAND_STOP)
                {
                    if (cornerFrequency.size() != 2)
                        std::cout << "Error. Bandpass or bandstop filter requires two corner frequencies to be specified" << std::endl;

                    this->samplingRate = samplingRate;
                    this->nTaps = nTaps;

                    double fcHigh = maxCornerFrequency / samplingRate;
                    double fcLow = minCornerFrequency / samplingRate;

                    // calculate filter coefficients
                    std::vector<double> coefficientHighPass = calculateCoefficients(fcHigh, HIGH_PASS, nTaps);
                    std::vector<double> coefficientLowPass = calculateCoefficients(fcLow, LOW_PASS, nTaps);

                    coefficients = std::vector<double>(coefficientHighPass.size(), 0);

                    for (int i = 0; i < coefficientHighPass.size(); i++)
                    {
                        if (LoHi == BAND_PASS)
                            coefficients[i] = -(coefficientHighPass[i] + coefficientLowPass[i]); //sum of HPF and LPF for bandstop filter, spectral inversion for bandpass filter
                        else
                            coefficients[i] = coefficientHighPass[i] + coefficientLowPass[i]; //sum of HPF and LPF for bandstop filter
                    }

                    if (LoHi == BAND_PASS)
                    {
                        //coefficients[(nTaps/2)+1] = coefficients[(nTaps/2)+1] +1;
                        coefficients[(nTaps / 2)] = coefficients[(nTaps / 2)] + 1;
                    }

                    this->validparameters = true;
                }
                else
                    std::cout << "Error. Undefined filter type: use 0 - lowpass, 1 - highpass, 2- bandpass, or 3- bandstop" << std::endl;
            }
        }

        // Note: start is inclusive, end is exclusive (as is conventional
        // in computer science)
        template <typename T>
        void Fill(T &array, int start, int end, double value)
        {
            if (array.empty())
            {
                std::cout << "empty vector" << std::endl;
            }
            if (start < 0 || start >= end)
            {
                std::cout << "fromIndex" << std::endl;
            }
            if (end > array.size())
            {
                std::cout << "toIndex" << std::endl;
            }
            for (int i = start; i < end; i++)
            {
                array[i] = value;
            }
        }

        double filterData(double data)
        {
            double dataFiltered = 0;

            if (!this->validparameters)
                std::cout << "Error. Filter parameters are invalid. Please set filter parameters before filtering data." << std::endl;
            else
            {
                int nSamples = 1;
                int bufferSize = this->nTaps;
                if (bufferedX.empty())
                {
                    bufferedX = std::vector<double>(bufferSize + nSamples, 0); // buffers are initiliazed to 0 by default
                                                                               //Arrays.fill(bufferedX, data); // fill the buffer X with the first data

                    Fill(bufferedX, 0, bufferedX.size(), data);
                }
                else
                {
                    //System.arraycopy(bufferedX, 1, bufferedX, 0, bufferedX.size()-1); //all the elements in the buffer are shifted one position to the left
                    // Array.Copy(bufferedX, 1, bufferedX, 0, bufferedX.size() - 1);
                    if (!bufferedX.empty())
                        bufferedX.erase(bufferedX.begin());
                    if (!bufferedX.empty())
                        *(bufferedX.end() - 1) = data;
                }

                double Y = filter(bufferedX);
                dataFiltered = Y;
            }

            return dataFiltered;
        }

        auto filterData(std::vector<double> &data)
        {
            if (!this->validparameters)
                //throw new Exception("Error. Filter parameters are invalid. Please set filter parameters before filtering data.");
                std::cout << "Error. Filter parameters are invalid. Please set filter parameters before filtering data." << std::endl;
            else
            {
                std::vector<double> dataFiltered(data.size(), 0);

                for (int i = 0; i < data.size(); i++)
                {
                    double individualDataFiltered = filterData(data[i]);
                    dataFiltered[i] = individualDataFiltered;
                }

                return dataFiltered;
            }
        }

        double GetSamplingRate()
        {
            return samplingRate;
        }

        auto GetCornerFrequency()
        {
            return cornerFrequency;
        }

        // static auto fromListToArray(List<Double> list)
        // {

        //     double[] array = new double[list.Count];
        //     for (int i = 0; i < list.Count; i++)
        //         array[i] = list[i];

        //     return array;
        // }

        void resetBuffers()
        {

            bufferedX.clear();
        }

        int getFilterType()
        {
            return filterType;
        }

    protected:
        void SamplingRate(double samplingrate)
        {
            samplingRate = samplingrate;
        }

        void SetCornerFrequency(std::vector<double> &cf)
        {
            cornerFrequency = cf;
        }

    private:
        double filter(std::vector<double> &X)
        {

            int nTaps = coefficients.size();
            double Y = 0;

            for (int i = 0; i < nTaps; i++)
                Y += X[nTaps - i] * coefficients[i];

            return Y;
        }

        std::vector<double> calculateCoefficients(double fc, int LoHi, int nTaps)
        {
            if (!(LoHi == LOW_PASS || LoHi == HIGH_PASS))
                std::cout << "Error: the function calculateCoefficients() can only be called for LPF or HPF" << std::endl;

            //Initialization
            int M = nTaps;
            std::vector<double> h(M, 0);

            for (int i = 0; i < M; i++)
            {
                h[i] = 0.42 - 0.5 * std::cos((2 * M_PI * i) / M) + 0.08 * std::cos((4 * M_PI * i) / M);
                if (i != M / 2)
                    h[i] = h[i] * (std::sin(2 * M_PI * fc * (i - (M / 2)))) / (i - (M / 2));
                else
                    h[i] = h[i] * (2 * M_PI * fc);
            }

            double gain = 0;
            for (int i = 0; i < h.size(); i++)
                gain += h[i];

            for (int i = 0; i < h.size(); i++)
            {
                if (LoHi == HIGH_PASS)
                {
                    h[i] = -h[i] / gain;
                }
                else
                    h[i] = h[i] / gain;
            }

            if (LoHi == HIGH_PASS)
            {
                h[M / 2] = h[M / 2] + 1;
            }

            return h;
        }

        // filter parameters
        int filterType;
        double samplingRate = 0; // Double.Nan
        std::vector<double> cornerFrequency;
        int nTaps;
        double minCornerFrequency, maxCornerFrequency;

        // buffered data (for filtering streamed data)
        std::vector<double> bufferedX;

        // filter coefficients {h}
        std::vector<double> coefficients;

        // input parameters are invalid
        bool validparameters = false;

        // default parameters
        double defaultSamplingRate = 512;
        std::vector<double> defaultCornerFrequency = {0.5};
        int defaultNTaps = 200;

    }; // Filter class

} // Shimmer3 namespace

#endif // _FILTER_HPP