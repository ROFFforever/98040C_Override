import os
from google import genai

# Initialize the client with your free API key
client = genai.Client(api_key='YOUR_API_KEY')

# Generate a response using the fast, free Gemini 2.5 Flash model
response = client.models.generate_content(
    model='gemini-2.5-flash',
    contents='Explain quantum computing in one simple sentence.',
)

# Print the output text
print(response.text)
