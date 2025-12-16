from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional, Dict
from ...agents.skills import AgentSkills

router = APIRouter(prefix="/agent", tags=["agent"])
agent_skills = AgentSkills()

class SkillRequest(BaseModel):
    skill: str
    text: str
    context: Optional[Dict[str, str]] = {}

@router.post("/skill")
async def execute_skill(request: SkillRequest):
    try:
        if request.skill == "translate_urdu":
            return {"result": agent_skills.translate_to_urdu(request.text)}
        elif request.skill == "personalize":
            return {"result": agent_skills.personalize_content(request.text, request.context)}
        elif request.skill == "summarize":
            return {"result": agent_skills.summarize_text(request.text)}
        else:
            raise HTTPException(status_code=400, detail="Unknown skill")
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
