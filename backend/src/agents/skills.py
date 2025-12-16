from typing import List, Dict, Any
import google.generativeai as genai
from ..config.settings import settings

class AgentSkills:
    def __init__(self):
        if settings.gemini_api_key:
            genai.configure(api_key=settings.gemini_api_key)
            self.model = genai.GenerativeModel('gemini-pro')
        else:
            self.model = None

    def _generate(self, prompt: str) -> str:
        if not self.model:
            return "Error: Gemini API Key not configured."
        try:
            response = self.model.generate_content(prompt)
            return response.text
        except Exception as e:
            return f"Error generating content: {str(e)}"

    def summarize_text(self, text: str, max_words: int = 100) -> str:
        """
        Skill: Summarize a given text.
        """
        prompt = f"Summarize the following text in under {max_words} words. Capture the key technical concepts.\n\nText: {text}"
        return self._generate(prompt)

    def translate_to_urdu(self, text: str) -> str:
        """
        Skill: Translate technical text to Urdu.
        """
        prompt = f"You are a technical translator. Translate the following text to Urdu. Keep technical terms (like 'Node', 'Topic', 'ROS 2', 'LLM') in English script or commonly understood Urdu transliteration. Ensure the tone is professional and academic.\n\nText: {text}"
        return self._generate(prompt)

    def personalize_content(self, text: str, user_profile: Dict[str, str]) -> str:
        """
        Skill: Rewrite content based on user's background.
        """
        software_bg = user_profile.get("software_bg", "General")
        hardware_bg = user_profile.get("hardware_bg", "General")

        prompt = f"""
        Restyle the explanation for a student with the following background:
        - Software: {software_bg}
        - Hardware: {hardware_bg}

        If they are beginners, use simple analogies.
        If they are advanced, use technical comparisons.

        Text: {text}
        """
        return self._generate(prompt)

    def generate_quiz(self, topic: str, difficulty: str = "medium") -> str:
        """
        Skill: Generate a quiz for a topic.
        """
        prompt = f"Generate 3 {difficulty} multiple-choice questions about {topic}. Return the output as a JSON-like string."
        return self._generate(prompt)
