import pyttsx3

def get_available_voices():
    """Print a list of available voices on the system."""
    engine = pyttsx3.init()
    voices = engine.getProperty('voices')

    print("Available Voices:")
    for voice in voices:
        print(f"ID: {voice.id}, Name: {voice.name}, Languages: {voice.languages}")

# Call the function to get available voices
get_available_voices()