#include <cstdio>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <deque>

#include "filt.h"

const double WWV_SUBCARRIER_HZ = 100.0;
const int SAMPLE_RATE = 8000;
const int GOERTZEL_M = SAMPLE_RATE * 0.01; // number of samples corresponding to a 0 or 1 for the carrier
const double GOERTZEL_K = WWV_SUBCARRIER_HZ * GOERTZEL_M / (float)SAMPLE_RATE;
const double GOERTZEL_COEFF = 2.0 * cos((2.0 * M_PI * WWV_SUBCARRIER_HZ) / (float)SAMPLE_RATE);
const double GOERTZEL_COEFF_IMAG = sin((2.0 * M_PI * WWV_SUBCARRIER_HZ) / (float)SAMPLE_RATE);
const double MIN_POWER_FOR_1 = 2e8;

std::deque<short> samplesSeen;
std::deque<double> windowCoeffs;
int phasesRemaining = GOERTZEL_M;
bool lookingForPhase = true;

enum 
{
    WAITING_FOR_BEGINNING, // Haven't seen the reference marker yet
    WAITING_FOR_DATA,      // Data bits in between position markers 
                           // (8 if we came from WAITING_FOR_BEGINNING, 
                           // 9 otherwise).
    WAITING_FOR_POSITION,  // Waiting for position marker
} currentState = WAITING_FOR_BEGINNING;
int dataBitsRemaining = 0;
int positionsRemaining = 0;

//=========================================================
// Predefined vectors indicating the possible "characters"
// WWV/WWVH can send using the 100 Hz subcarrier.
//
// Note: each 1 and 0 below is 10ms (see GOERTZEL_M above),
// making each row 70-100ms long. 1 indicates the presence
// of the 100 Hz carrier, 0 otherwise. Characters are defined
// beginning from when the carrier becomes high until when
// the next one should be present.
//=========================================================
std::deque<char> ReferenceMarker = {
    // 0.770s position identifier (P0)
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1,
    
    // Additional zeros so that the hole below starts at next
    // second exactly (1.000 - 0.030 - 0.770 = 0.200s)
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    
    // 1.030s "hole"
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0,
};

std::deque<char> ReferenceMarkerProcessed;

std::deque<char> PositionMarker = {
    // 0.770s position identifier (P1-P5)
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1,
    
    // 0.230s gap before next character
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0,
};

std::deque<char> PositionMarkerProcessed;

std::deque<char> OneBit = {
    // 0.470s position identifier
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 
    
    // 0.530s gap before next character
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0,
};

std::deque<char> OneBitProcessed;

std::deque<char> ZeroBit = {
    // 0.170s position identifier
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1,
    
    // 0.830s gap before next character
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0,
};

std::deque<char> ZeroBitProcessed;

bool fuzzyMatch(std::deque<char>& one, std::deque<char>& other)
{
    // Returns true if <= 12% of items in both don't match.
    // Note: 12% is from experimentation based on OTA recordings of WWV.
    int maxFailures = std::min(one.size(), other.size()) * 0.12;
    int numFailures = 0;
    
    for (std::deque<char>::iterator i = one.begin(), j = other.begin();
         i != one.end() && j != other.end();
         i++, j++)
     {
         if (*i != *j)
         {
             numFailures++;
             if (numFailures > maxFailures) return false;
         }
     }
     
     return true;
}

std::deque<char> carriersSeen;

#include <q/fx/envelope.hpp>
#include <q/fx/dynamic.hpp>
#include <q/fx/noise_gate.hpp>
#include <q/fx/dc_block.hpp>
#include <q/fx/moving_average.hpp>

using namespace cycfi::q::literals;

cycfi::q::fast_ave_envelope_follower follower(2_ms, SAMPLE_RATE);
cycfi::q::noise_gate ng(-32.5_dB);
cycfi::q::dc_block dcBlocker(60_Hz, SAMPLE_RATE);
cycfi::q::moving_average noiseAvg(200_ms, SAMPLE_RATE);

int sampCount = -1;
int sampSum = 0;

void processIncomingSample(short sample)
{
    auto floatSample = (float)sample / SHRT_MAX;    
    auto followOut = follower(floatSample);
    
    // Adjust moving average. Don't adjust thresholds yet.
    // That will be done whenever we have enough samples to
    // attempt a symbol decode.
    noiseAvg(followOut);
    
    //auto gainVal = gc(followOut, 15_dB);
    auto gateVal = ng(followOut);
    
    //std::cout << (gateVal ? 1 : 0);
    carriersSeen.push_back(gateVal ? 1 : 0);
    
    if (carriersSeen[0] == 0)
    {
        // We should match on the first 1 we see to make the 
        // rest of the decode logic work reliably.
        carriersSeen.pop_front();
    }
}

#if 0
std::deque<char> processIncomingSample(short sample)
{
    if (windowCoeffs.size() == 0)
    {
        for (int i = 0; i < GOERTZEL_M; i++)
        {
            double tmp = sin(M_PI * i / GOERTZEL_M);
            windowCoeffs.push_back(tmp * tmp);
        }
    }
    
    std::deque<char> result;
    samplesSeen.push_back(sample);
    if (samplesSeen.size() > GOERTZEL_M * ReferenceMarker.size())
    {
        // Only keep enough samples to handle a reference marker.
        samplesSeen.pop_front();
        
        // If we couldn't get a lock on the signal by checking all GOERTZEL_M
        // starting points, we should just clear the remaining samples that would have
        // been part of the reference marker and try again next time around. This
        // is hopefully a reasonable optimization to allow this to run in real time.
        if (lookingForPhase)
        {
            phasesRemaining--;
            if (phasesRemaining == 0)
            {
                phasesRemaining = GOERTZEL_M;
                for (int i = 0; samplesSeen.size() > 0 && i < GOERTZEL_M * ReferenceMarker.size() - phasesRemaining; i++)
                {
                    samplesSeen.pop_front();
                }
            }
        }
    }
    
    if ((currentState == WAITING_FOR_BEGINNING && samplesSeen.size() < GOERTZEL_M * ReferenceMarker.size()) ||
        (samplesSeen.size() < GOERTZEL_M * OneBit.size()))
    {
        // Not enough samples yet, don't bother running the rest of this method until we do.
        return result;
    }
    
    int index = 0;
    for (int sampleRun = 0; sampleRun < floor(samplesSeen.size() / GOERTZEL_M); sampleRun++)
    {
        int numSamplesSeen = 0;
        double u0 = 0;
        double u1 = 0;
        
        for (; numSamplesSeen < GOERTZEL_M; numSamplesSeen++, index++)
        {
            double sampleAsFloat = (double)samplesSeen[index] * windowCoeffs[numSamplesSeen]; // / SHRT_MAX;
            
            // Apply Hann window first
            //float hannCoeff = sin(M_PI * numSamplesSeen / GOERTZEL_M);
            //sampleAsFloat *= hannCoeff * hannCoeff; // sin^2
            
            double tmp = sampleAsFloat + (GOERTZEL_COEFF * u0) - u1;
            u1 = u0;
            u0 = tmp;
        }
        
        double resultMag = (u0 * u0) + (u1 * u1) - (GOERTZEL_COEFF * u0 * u1);
        //carrierPresent = power > MIN_POWER_FOR_1;
        //float resultR = 0.5 * GOERTZEL_COEFF * u0 - u1;
        //float resultI = GOERTZEL_COEFF_IMAG * u0;
        //float resultMag = resultR * resultR + resultI * resultI;
        
        //std::cout << resultMag << " ";
        result.push_back(resultMag > MIN_POWER_FOR_1 ? 1 : 0);
    }
    
    //std::cout << std::endl;
    
    // debug: print result before returning
    /*if (currentState != WAITING_FOR_BEGINNING)*/
    /*{
        for (auto& x : result)
        {
            if (x) std::cout << "1";
            else std::cout << "0";
        }
        std::cout << std::endl;
    }*/
    
    return result;
}
#endif

void parseTimeCode(std::deque<char>& timeCodeSeen)
{
    /*for (auto& x : timeCodeSeen)
    {
        std::cout << x << " ";
    }
    std::cout << std::endl;*/
    
    int year = 2000 + 
        (timeCodeSeen[4] == '1' ? 1 : 0) +
        (timeCodeSeen[5] == '1' ? 2 : 0) +
        (timeCodeSeen[6] == '1' ? 4 : 0) +
        (timeCodeSeen[7] == '1' ? 8 : 0);
    
    year += timeCodeSeen[51] == '1' ? 10 : 0;
    year += timeCodeSeen[52] == '1' ? 20 : 0;
    year += timeCodeSeen[53] == '1' ? 40 : 0;
    year += timeCodeSeen[54] == '1' ? 80 : 0;
    
    int days = 
        (timeCodeSeen[30] == '1' ? 1 : 0) +
        (timeCodeSeen[31] == '1' ? 2 : 0) +
        (timeCodeSeen[32] == '1' ? 4 : 0) +
        (timeCodeSeen[33] == '1' ? 8 : 0) +
            
        (timeCodeSeen[35] == '1' ? 10 : 0) +
        (timeCodeSeen[36] == '1' ? 20 : 0) +
        (timeCodeSeen[37] == '1' ? 40 : 0) +
        (timeCodeSeen[38] == '1' ? 80 : 0) +
            
        (timeCodeSeen[40] == '1' ? 100 : 0) +
        (timeCodeSeen[41] == '1' ? 200 : 0);
    
    std::cout << "Date: " << "Day " << days << " of year " << year << std::endl;
    
    int hours = 
        (timeCodeSeen[20] == '1' ? 1 : 0) +
        (timeCodeSeen[21] == '1' ? 2 : 0) +
        (timeCodeSeen[22] == '1' ? 4 : 0) +
        (timeCodeSeen[23] == '1' ? 8 : 0) +
            
        (timeCodeSeen[25] == '1' ? 10 : 0) +
        (timeCodeSeen[26] == '1' ? 20 : 0);
    
    int minutes = 
        (timeCodeSeen[10] == '1' ? 1 : 0) +
        (timeCodeSeen[11] == '1' ? 2 : 0) +
        (timeCodeSeen[12] == '1' ? 4 : 0) +
        (timeCodeSeen[13] == '1' ? 8 : 0) +
            
        (timeCodeSeen[15] == '1' ? 10 : 0) +
        (timeCodeSeen[16] == '1' ? 20 : 0) +
        (timeCodeSeen[17] == '1' ? 40 : 0);
    
    std::cout << "Time (UTC): " << hours << ":" << std::setfill('0') << std::setw(2) << minutes << std::endl;
}

void processMarkers(std::deque<char>& orig, std::deque<char>& processed)
{
    // Converts 10ms blocks into 1ms blocks.
    for (auto& item : orig)
    {
        for (int index = 0; index < GOERTZEL_M; index++)
        {
            processed.push_back(item);
        }
    }
}

int main()
{
    short sampleShort = 0;
    std::deque<char> timeCodeSeen;
    
    /*cycfi::q::agc gc(37_dB);
    cycfi::q::noise_gate ng(-52.5_dB);*/
    cycfi::q::fast_rms_envelope_follower_db follower(1_s, SAMPLE_RATE);
    cycfi::q::moving_average volAvg(1_s, SAMPLE_RATE);

    processMarkers(ReferenceMarker, ReferenceMarkerProcessed);
    processMarkers(PositionMarker, PositionMarkerProcessed);
    processMarkers(OneBit, OneBitProcessed);
    processMarkers(ZeroBit, ZeroBitProcessed);
    
    Filter filt(BPF, 255, SAMPLE_RATE, 75, 150);
    //Filter filt(BPF, 1000, SAMPLE_RATE, 95, 105);
    if (filt.get_error_flag() != 0)
    {
        std::cout << "Filter error: " << filt.get_error_flag() << std::endl;
    }

    FILE* fp = fopen("tmp.raw", "wb");
    assert(fp != nullptr);
    
    while (fread((void*)&sampleShort, sizeof(short), 1, stdin) > 0)
    {
        // block DC
        float blockedAudio = dcBlocker((double)sampleShort / SHRT_MAX);
        
        sampleShort = (short)filt.do_sample(blockedAudio * SHRT_MAX);
        
        auto followedEnv = follower((double)sampleShort / SHRT_MAX);
        auto dbRequiredtoAdd = -6_dB + -followedEnv;
        auto multiplier = cycfi::q::lin_double(dbRequiredtoAdd);
        //auto multiplier = cycfi::q::lin_double(-6_dB) / volAvg(cycfi::q::lin_double(followedEnv));
        
        //auto multiplier = 1 - abs(avg); //lin_double(-cycfi::q::lin_to_db(-avg));
        //std::cout << multiplier << std::endl;
        sampleShort *= multiplier;
        
        fwrite(&sampleShort, sizeof(short), 1, fp);
        
        // Envelope
        /*auto envOut = follower((double)sampleShort / SHRT_MAX);

        // Noise gate -- mute anything under it so we don't
        // get any false positives later.
        if (ng(lin_float(envOut)) == 0)
        {
            sampleShort = 0;
        }
        
        // AGC
        auto gainDB = gc(envOut, -6_dB);
        sampleShort *= lin_float(gainDB);*/
        
        //std::cout << "multiplier: " << lin_float(gainDB) << std::endl;
        
        //carriersSeen = processIncomingSample(sampleShort);
        processIncomingSample(sampleShort);
        
        bool adjustNoiseGate = false;
        
        switch (currentState)
        {
            case WAITING_FOR_BEGINNING:
                if (carriersSeen.size() == ReferenceMarkerProcessed.size())
                {
                    adjustNoiseGate = true;
                    
                    int numCarriersToPop = 0;
                    if (fuzzyMatch(carriersSeen, ReferenceMarkerProcessed))
                    {
                        // Seen reference marker, now listen for first data bits
                        dataBitsRemaining = 8;
                        positionsRemaining = 5; // Expecting five more position bits
                        currentState = WAITING_FOR_DATA;
                    
                        std::cout << std::endl;
                        timeCodeSeen.push_back('R');
                        std::cout << "R";
                        
                        // Since we're sync'd up with the radio now, we can shortcut the
                        // rest of the matching.
                        numCarriersToPop = carriersSeen.size();
                        lookingForPhase = false;
                    }
                    else if (lookingForPhase && (
                        fuzzyMatch(carriersSeen, OneBitProcessed) || 
                        fuzzyMatch(carriersSeen, ZeroBitProcessed) ||
                        fuzzyMatch(carriersSeen, ReferenceMarkerProcessed)))
                    {
                        // Another way we can shortcut the phase search is finding 1, 0 or P.
                        // However, we still need to find R to start being able to read the time.
                        std::cout << "Locked onto WWV signal" << std::endl;
                        numCarriersToPop = OneBitProcessed.size();
                        lookingForPhase = false;
                    }
                    else
                    {
                        // The channel was too noisy to receive the reference marker
                        // (or we started listening in the middle of a time code).
                        // Only pop the beginning of the list in case of the latter.
                        //samplesSeen.pop_front();
                        carriersSeen.pop_front();
                    }
                    
                    if (!lookingForPhase)
                    {
                        for (int i = 0; i < numCarriersToPop; i++)
                        {
                            carriersSeen.pop_front();
                            //samplesSeen.pop_front();
                        }
                    }
                }
                break;
            case WAITING_FOR_DATA:
                if (carriersSeen.size() == OneBitProcessed.size()) // ZeroBit is the same size, or should be anyway
                {
                    adjustNoiseGate = true;
                    
                    bool found = false;
                    if (fuzzyMatch(carriersSeen, OneBitProcessed))
                    {
                        timeCodeSeen.push_back('1');
                        std::cout << "1";
                        found = true;
                    }
                    else if (fuzzyMatch(carriersSeen, ZeroBitProcessed))
                    {
                        timeCodeSeen.push_back('0');
                        std::cout << "0";
                        found = true;
                    }
                    
                    if (found)
                    {
                        dataBitsRemaining--;
                        if (dataBitsRemaining == 0)
                        {
                            if (positionsRemaining == 0)
                            {
                                // We should have a full timecode now, print it out for now
                                // TBD: do other things with it (e.g. generate Unix timestamp, inject into chrony)
                                //std::cout << (const char*)&timeCodeSeen << std::endl;
                                std::cout << std::endl;
                                parseTimeCode(timeCodeSeen);
                                timeCodeSeen.clear();
                                
                                currentState = WAITING_FOR_BEGINNING;
                            }
                            else
                            {
                                // We need to see another position marker now
                                currentState = WAITING_FOR_POSITION;
                            }
                        }
                    }
                    else
                    {
                        // We lost the WWV signal, so wait for another reference marker
                        currentState = WAITING_FOR_BEGINNING;
                        std::cout << std::endl << "lost sync during data wait" << std::endl;
                        /*
                        std::cout << "Expected (0): ";
                        for (auto& x : ZeroBitProcessed)
                        {
                            std::cout << (x ? "1" : "0");
                        }
                        std::cout << std::endl;
                        std::cout << "Expected (1): ";
                        for (auto& x : OneBitProcessed)
                        {
                            std::cout << (x ? "1" : "0");
                        }
                        std::cout << std::endl;
                        std::cout << "Actual:       ";
                        for (auto& x : carriersSeen)
                        {
                            std::cout << (x ? "1" : "0");
                        }
                        std::cout << std::endl;
                        std::cout << "file idnex " << ftell(stdin) << std::endl;
                        */
                        timeCodeSeen.clear();
                    }
                    
                    //samplesSeen.clear();
                    carriersSeen.clear();
                }
                break;
            case WAITING_FOR_POSITION:
                if (carriersSeen.size() == PositionMarkerProcessed.size())
                {
                    adjustNoiseGate = true;
                    
                    if (fuzzyMatch(carriersSeen, PositionMarkerProcessed))
                    {
                        dataBitsRemaining = 9;
                        positionsRemaining--;
                        currentState = WAITING_FOR_DATA;
                
                        timeCodeSeen.push_back('P');
                        std::cout << "P";
                    }
                    else
                    {
                        // We lost the WWV signal, so wait for another reference marker
                        std::cout << std::endl << "lost sync during position wait" << std::endl;
                        currentState = WAITING_FOR_BEGINNING;
                        timeCodeSeen.clear();
                    }
                    
                    //samplesSeen.clear();
                    carriersSeen.clear();
                }
                break;
        };
        
        if (adjustNoiseGate)
        {
            // Adjust noise gate threshold for next go-around.
            auto noiseLevelDB = cycfi::q::lin_to_db(noiseAvg());
            ng.release_threshold(noiseLevelDB);
            //ng.onset_threshold(noiseLevelDB + 6_dB); // Copies what the constructor does.
        }
    }
    
    fclose(fp);
    
    return 0;
}
