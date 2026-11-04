from pathlib import Path

from hw_semantic_components.cia402 import decode_state

CASES = Path(__file__).with_name("cia402_cases.txt")


def test_python_decoder_matches_the_shared_cases():
    for line in CASES.read_text().splitlines():
        if line and not line.startswith("#"):
            word, state = line.split()
            assert decode_state(int(word, 16)) == state, word
