import pygame
import numpy as np
import time

def play_tone(frequency, duration):
    """Play a tone of specified frequency for the specified duration."""
    pygame.mixer.init(44100, -16, 2, 1024)  # Changed to stereo (2 channels)
    sample_rate = 44100
    
    # Generate a proper sine wave at the correct frequency
    # Calculate the number of samples needed for the duration
    n_samples = int(round(duration * sample_rate))
    
    # Create time array
    t = np.linspace(0, duration, n_samples, False)
    
    # Generate sine wave
    note = np.sin(2 * np.pi * frequency * t)
    
    # Convert to 16-bit data
    audio = note * 32767 / 2  # Half volume
    audio = audio.astype(np.int16)
    
    # Convert to stereo by duplicating the mono channel
    stereo_audio = np.column_stack((audio, audio))
    
    # Start playback
    sound = pygame.sndarray.make_sound(stereo_audio)
    sound.play()
    
    # Wait for the sound to finish
    time.sleep(duration)
    sound.stop()

def play_star_trek_theme():
    """Plays a simplified version of the Star Trek theme"""
    print("Playing Star Trek theme melody...")
    
    # Define notes frequencies (Hz)
    AS3 = 233.08  # A sharp
    B3 = 246.94
    C4 = 261.63   # C
    D4 = 293.66
    CS4 = 277.18  # C sharp
    DS4 = 311.13  # D sharp
    E4 = 329.63
    F4 = 349.23   # F
    FS3 = 185.00  # F sharp (one octave lower)
    FS4 = 369.99  # F sharp
    G4 = 392.00   # G
    A4 = 440.00
    A3 = 220.00   # A3
    B4 = 493.88   # B
    C5 = 523.25   # C (higher octave)

    # Define note durations (seconds)
    QUARTER = 0.25
    HALF = 0.5
    WHOLE = 1.0
    TRIPLET = 0.33  # One note in a triplet (3 notes in the time of 1 beat)
    
    # C major scale sequence
    c_major_scale = [
        (C4, QUARTER),   # C
        (D4, QUARTER),   # D
        (E4, QUARTER),   # E
        (F4, QUARTER),   # F
        (G4, QUARTER),   # G
        (A4, QUARTER),   # A
        (B4, QUARTER),   # B
        (C5, QUARTER),   # C (higher octave)
    ]
    
    # Play C major scale
    print("Playing C major scale...")
    for note, duration in c_major_scale:
        play_tone(note, duration)
        time.sleep(0.05)  # Brief pause between notes
    
    print("Now playing Star Trek theme...")
    time.sleep(0.5)  # Short pause between sequences

    # New sequence as specified
    sequence = [
        (FS3, HALF),           # F#3 (HALF) - corrected to lower octave
        (E4, HALF + QUARTER),  # E (HALF + quarter)
        (DS4, QUARTER),        # D# (quarter)
        (CS4, TRIPLET),        # C# (triplet note 1)
        (B3, TRIPLET),         # B  (triplet note 2)
        (AS3, TRIPLET),        # A# (triplet note 3)
        (A3, WHOLE),           # A (WHOLE)
    ]

    # Play the sequence
    for note, duration in sequence:
        play_tone(note, duration)
        time.sleep(0.1)  # Brief pause between notes
        
    print("Finished playing.")

if __name__ == "__main__":
    pygame.init()
    try:
        play_star_trek_theme()
    finally:
        pygame.quit()