from typing import List, Dict, Any
import openai
from ..config.settings import settings

class AgentSkills:
    def __init__(self):
        self.client = openai.OpenAI(api_key=settings.openai_api_key)

    def summarize_text(self, text: str, max_words: int = 100) -> str:
        """
        Skill: Summarize a given text.
        """
        response = self.client.chat.completions.create(
            model=settings.model_name,
            messages=[
                {"role": "system", "content": f"Summarize the following text in under {max_words} words. Capture the key technical concepts."},
                {"role": "user", "content": text}
            ]
        )
        return response.choices[0].message.content

    def translate_to_urdu(self, text: str) -> str:
        """
        Skill: Translate technical text to Urdu, keeping technical terms in English or transliterated where appropriate.
        """
        response = self.client.chat.completions.create(
            model=settings.model_name,
            messages=[
                {"role": "system", "content": "You are a technical translator. Translate the following text to Urdu. Keep technical terms (like 'Node', 'Topic', 'ROS 2', 'LLM') in English script or commonly understood Urdu transliteration. Ensure the tone is professional and academic."},
                {"role": "user", "content": text}
            ]
        )
        return response.choices[0].message.content

    def personalize_content(self, text: str, user_profile: Dict[str, str]) -> str:
        """
        Skill: Rewrite content based on user's background.
        """
        software_bg = user_profile.get("software_bg", "General")
        hardware_bg = user_profile.get("hardware_bg", "General")

        system_prompt = f"""
        Restyle the explanation for a student with the following background:
        - Software: {software_bg}
        - Hardware: {hardware_bg}

        If they are beginners, use simple analogies.
        If they are advanced, use technical comparisons (e.g., compare ROS nodes to Microservices).
        """

        response = self.client.chat.completions.create(
            model=settings.model_name,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": text}
            ]
        )
        return response.choices[0].message.content

    def generate_quiz(self, topic: str, difficulty: str = "medium") -> List[Dict[str, Any]]:
        """
        Skill: Generate a quiz for a topic.
        """
        # simplified implementation returning a structured string for now, could be JSON
        response = self.client.chat.completions.create(
            model=settings.model_name,
            messages=[
                {"role": "system", "content": f"Generate 3 {difficulty} multiple-choice questions about {topic}. Return the output as a JSON-like string."},
                {"role": "user", "content": "Generate the quiz."}
            ]
        )
        return response.choices[0].message.content
