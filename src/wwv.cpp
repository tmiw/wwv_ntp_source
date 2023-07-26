#include <cstdio>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <deque>

#include <q/fx/envelope.hpp>
#include <q/fx/dynamic.hpp>
#include <q/fx/noise_gate.hpp>
#include <q/fx/dc_block.hpp>
#include <q/fx/moving_average.hpp>

#include "filt.h"

using namespace cycfi::q::literals;

const int SAMPLE_RATE = 8000;
const int NUM_BLOCKS_PER_10_MS = SAMPLE_RATE * 0.01; // number of samples corresponding to a 0 or 1 for the carrier

cycfi::q::fast_ave_envelope_follower follower(2_ms, SAMPLE_RATE);
cycfi::q::noise_gate ng(-32.5_dB);
cycfi::q::dc_block dcBlocker(60_Hz, SAMPLE_RATE);
cycfi::q::moving_average noiseAvg(200_ms, SAMPLE_RATE);

std::deque<char> carriersSeen;
std::deque<double> windowCoeffs;
int phasesRemaining = NUM_BLOCKS_PER_10_MS;
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
// Note: each 1 and 0 below is 10ms, making each row 70-100ms 
// long. 1 indicates the presence of the 100 Hz carrier, 0 
// otherwise. Characters are defined beginning from when 
// the carrier becomes high until when the next one should 
// be present.
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

void processIncomingSample(short sample)
{
    auto floatSample = (float)sample / SHRT_MAX;    
    auto followOut = follower(floatSample);
    
    // Adjust moving average. Don't adjust thresholds yet.
    // That will be done whenever we have enough samples to
    // attempt a symbol decode.
    noiseAvg(followOut);
    
    auto gateVal = ng(followOut);
    
    carriersSeen.push_back(gateVal ? 1 : 0);
    
    if (carriersSeen[0] == 0)
    {
        // We should match on the first 1 we see to make the 
        // rest of the decode logic work reliably.
        carriersSeen.pop_front();
    }
}

void parseTimeCode(std::deque<char>& timeCodeSeen)
{
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
        for (int index = 0; index < NUM_BLOCKS_PER_10_MS; index++)
        {
            processed.push_back(item);
        }
    }
}

int main()
{
    short sampleShort = 0;
    std::deque<char> timeCodeSeen;
    
    cycfi::q::fast_rms_envelope_follower_db follower(1_s, SAMPLE_RATE);
    cycfi::q::moving_average volAvg(1_s, SAMPLE_RATE);

    processMarkers(ReferenceMarker, ReferenceMarkerProcessed);
    processMarkers(PositionMarker, PositionMarkerProcessed);
    processMarkers(OneBit, OneBitProcessed);
    processMarkers(ZeroBit, ZeroBitProcessed);
    
    Filter filt(BPF, 255, SAMPLE_RATE, 75, 150);
    if (filt.get_error_flag() != 0)
    {
        std::cout << "Filter error: " << filt.get_error_flag() << std::endl;
    }

    //FILE* fp = fopen("tmp.raw", "wb");
    //assert(fp != nullptr);
    
    while (fread((void*)&sampleShort, sizeof(short), 1, stdin) > 0)
    {
        // Block DC and amplify signal so that the decoder can pick it up.
        float blockedAudio = dcBlocker((double)sampleShort / SHRT_MAX);
        
        sampleShort = (short)filt.do_sample(blockedAudio * SHRT_MAX);
        
        auto followedEnv = follower((double)sampleShort / SHRT_MAX);
        auto dbRequiredtoAdd = -6_dB + -followedEnv;
        auto multiplier = cycfi::q::lin_double(dbRequiredtoAdd);

        sampleShort *= multiplier;
        
        //fwrite(&sampleShort, sizeof(short), 1, fp);
        
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
                        carriersSeen.pop_front();
                    }
                    
                    if (!lookingForPhase)
                    {
                        for (int i = 0; i < numCarriersToPop; i++)
                        {
                            carriersSeen.pop_front();
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
                        lookingForPhase = true;
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
                        
                        lookingForPhase = false;
                    }
                    
                    carriersSeen.clear();
                }
                break;
        };
        
        if (adjustNoiseGate)
        {
            // Adjust noise gate threshold for next go-around.
            auto noiseLevelDB = cycfi::q::lin_to_db(noiseAvg());
            ng.release_threshold(noiseLevelDB);
        }
        
        fflush(stdout);
    }
    
    //fclose(fp);
    
    return 0;
}
