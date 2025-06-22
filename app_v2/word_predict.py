import unidecode

def load_vietnamese_dictionary(file_path):
    """Load words from a Vietnamese dictionary file."""
    with open(file_path, "r", encoding="utf-8") as file:
        words = file.read().splitlines()  # Read all lines and remove trailing newlines
    return words

def remove_tones(word):
    """Remove tone marks from a Vietnamese word."""
    return unidecode.unidecode(word)

def get_closest_words(prefix, dictionary, num_suggestions=5):
    """Get the closest words that start with the given prefix (ignoring tones)."""
    prefix_no_tones = remove_tones(prefix)  # Remove tones from the input prefix
    suggestions = []
    
    for word in dictionary:
        if remove_tones(word).startswith(prefix_no_tones):  # Remove tones and compare
            suggestions.append(word)
            if len(suggestions) >= num_suggestions:
                break
    
    return suggestions

# Load the dictionary (replace with your dictionary file path)
file_path = "vietnamese.txt"  # Path to your Vietnamese dictionary file
vietnamese_dict = load_vietnamese_dictionary(file_path)

# Example usage
prefix = input("Nhập từ gợi ý: ")  # Example: "cha"
suggestions = get_closest_words(prefix, vietnamese_dict)

print("Gợi ý từ gần nhất:")
for suggestion in suggestions:
    print(suggestion)
