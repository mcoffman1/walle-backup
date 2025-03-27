import speech_recognition as sr

def recognize_speech_from_mic(recognizer, microphone):
    with microphone as source:
        print("Listening...")  # This will still print to the console
        recognizer.adjust_for_ambient_noise(source)  # Adjust for ambient noise
        audio = recognizer.listen(source)

    # Try to recognize the speech in the audio
    try:
        transcription = recognizer.recognize_google(audio)
    except sr.RequestError:
        transcription = "API unavailable"
    except sr.UnknownValueError:
        transcription = "Unable to recognize speech"
    
    return transcription

if __name__ == "__main__":
    recognizer = sr.Recognizer()
    mic = sr.Microphone(device_index=0)  # Adjust the device index based on your system

    # Open a file to write the transcriptions
    with open("transcriptions.txt", "a") as file:
        while True:
            transcription = recognize_speech_from_mic(recognizer, mic)
            print("Transcription added to file.")
            file.write(transcription + "\n")

