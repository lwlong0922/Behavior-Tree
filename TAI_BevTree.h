#ifndef __TAI_BEVTREE_H__
#define __TAI_BEVTREE_H__

#include <string>
#include "TUtility_AnyData.h"

namespace TsiU
{
	namespace AI
	{
		namespace BehaviorTree
		{

#define k_BLimited_MaxChildNodeCnt 16
#define k_BLimited_InvalidChildNodeIndex k_BLimited_MaxChildNodeCnt

			//并行完成条件判断
			enum E_ParallelFinishCondition
			{
				k_PFC_OR = 1,
				k_PFC_AND
			};

			//行为运行状态
			enum BevRunningStatus
			{
				k_BRS_Executing = 0,
				k_BRS_Finish = 1,
				k_BRS_ERROR_Transition = -1,
			};

			//行为节点状态
			enum E_TerminalNodeStaus
			{
				k_TNS_Ready = 1,
				k_TNS_Running = 2,
				k_TNS_Finish = 3,
			};

			typedef AnyData BevNodeInputParam;
			typedef AnyData BevNodeOutputParam;

			//---------------------------------------------------------------------------------------------------------------------------------

			//前提条件类（虚类）
			class BevNodePrecondition
			{
			public:
				virtual bool ExternalCondition(const BevNodeInputParam &input) const = 0;
			};

			//true
			class BevNodePreconditionTRUE : public BevNodePrecondition
			{
			public:
				virtual bool ExternalCondition(const BevNodeInputParam &input) const
				{
					return true;
				}
			};

			//false
			class BevNodePreconditionFALSE : public BevNodePrecondition
			{
			public:
				virtual bool ExternalCondition(const BevNodeInputParam &input) const
				{
					return false;
				}
			};

			//逻辑 非
			class BevNodePreconditionNOT : public BevNodePrecondition
			{
			public:
				BevNodePreconditionNOT(BevNodePrecondition *lhs)
					: m_lhs(lhs)
				{
					D_CHECK(m_lhs);
				}
				~BevNodePreconditionNOT()
				{
					D_SafeDelete(m_lhs);
				}
				virtual bool ExternalCondition(const BevNodeInputParam &input) const
				{
					return !m_lhs->ExternalCondition(input);
				}

			private:
				BevNodePrecondition *m_lhs;
			};
			
			//逻辑 与
			class BevNodePreconditionAND : public BevNodePrecondition
			{
			public:
				BevNodePreconditionAND(BevNodePrecondition *lhs, BevNodePrecondition *rhs)
					: m_lhs(lhs), m_rhs(rhs)
				{
					D_CHECK(m_lhs && m_rhs);
				}
				~BevNodePreconditionAND()
				{
					D_SafeDelete(m_lhs);
					D_SafeDelete(m_rhs);
				}
				virtual bool ExternalCondition(const BevNodeInputParam &input) const
				{
					return m_lhs->ExternalCondition(input) && m_rhs->ExternalCondition(input);
				}

			private:
				BevNodePrecondition *m_lhs;
				BevNodePrecondition *m_rhs;
			};
			
			//逻辑 或
			class BevNodePreconditionOR : public BevNodePrecondition
			{
			public:
				BevNodePreconditionOR(BevNodePrecondition *lhs, BevNodePrecondition *rhs)
					: m_lhs(lhs), m_rhs(rhs)
				{
					D_CHECK(m_lhs && m_rhs);
				}
				~BevNodePreconditionOR()
				{
					D_SafeDelete(m_lhs);
					D_SafeDelete(m_rhs);
				}
				virtual bool ExternalCondition(const BevNodeInputParam &input) const
				{
					return m_lhs->ExternalCondition(input) || m_rhs->ExternalCondition(input);
				}

			private:
				BevNodePrecondition *m_lhs;
				BevNodePrecondition *m_rhs;
			};
			
			//逻辑 亦或
			class BevNodePreconditionXOR : public BevNodePrecondition
			{
			public:
				BevNodePreconditionXOR(BevNodePrecondition *lhs, BevNodePrecondition *rhs)
					: m_lhs(lhs), m_rhs(rhs)
				{
					D_CHECK(m_lhs && m_rhs);
				}
				~BevNodePreconditionXOR()
				{
					D_SafeDelete(m_lhs);
					D_SafeDelete(m_rhs);
				}
				virtual bool ExternalCondition(const BevNodeInputParam &input) const
				{
					return m_lhs->ExternalCondition(input) ^ m_rhs->ExternalCondition(input);
				}

			private:
				BevNodePrecondition *m_lhs;
				BevNodePrecondition *m_rhs;
			};
			
			//---------------------------------------------------------------------------------------------------------------------------------

			//树节点基类
			class BevNode
			{
			public:
				BevNode(BevNode *_o_ParentNode, BevNodePrecondition *_o_NodeScript = NULL)
					: mul_ChildNodeCount(0), mz_DebugName("UNNAMED"), mo_ActiveNode(NULL), mo_LastActiveNode(NULL), mo_NodePrecondition(NULL)
				{
					for (int i = 0; i < k_BLimited_MaxChildNodeCnt; ++i)
						mao_ChildNodeList[i] = NULL;

					_SetParentNode(_o_ParentNode);
					SetNodePrecondition(_o_NodeScript);
				}

				virtual ~BevNode()
				{
					for (unsigned int i = 0; i < mul_ChildNodeCount; ++i)
					{
						D_SafeDelete(mao_ChildNodeList[i]);
					}
					D_SafeDelete(mo_NodePrecondition);
				}
				
				//评估 先判断是否满足当前节点的前提条件，满足的话再去做 _DoEvaluate(这里用于调用子类的_DoEvaluate)
				bool Evaluate(const BevNodeInputParam &input)
				{
					//&&前面的是控制节点本身的评估，后面的则会调用孩子的评估函数。
					return (mo_NodePrecondition == NULL || mo_NodePrecondition->ExternalCondition(input)) && _DoEvaluate(input);
				}
				
				//转移，从上一个可运行的节点切换到另一个节点的行为，如何取决于子类。
				void Transition(const BevNodeInputParam &input)
				{
					_DoTransition(input);
				}
				
				//更新，传入数据到行为节点进行更新
				BevRunningStatus Tick(const BevNodeInputParam &input, BevNodeOutputParam &output)
				{
					return _DoTick(input, output);
				}
				//---------------------------------------------------------------
				
				//添加孩子节点
				BevNode &AddChildNode(BevNode *_o_ChildNode)
				{
					//越界判断
					if (mul_ChildNodeCount == k_BLimited_MaxChildNodeCnt)
					{
						D_Output("The number of child nodes is up to 16");
						D_CHECK(0);
						return (*this);
					}
					mao_ChildNodeList[mul_ChildNodeCount] = _o_ChildNode;
					++mul_ChildNodeCount;
					return (*this);
				}
				
				//设置前提条件
				BevNode &SetNodePrecondition(BevNodePrecondition *_o_NodePrecondition)
				{
					//如果传入的先决条件和现在的一样就不做任何操作。
					if (mo_NodePrecondition != _o_NodePrecondition)
					{
						//如果本来就有先决条件，则先释放
						if (mo_NodePrecondition)
							delete mo_NodePrecondition;

						mo_NodePrecondition = _o_NodePrecondition;
					}
					return (*this);
				}

				//给节点设置名字
				BevNode &SetDebugName(const char *_debugName)
				{
					mz_DebugName = _debugName;
					return (*this);
				}
				
				//获取上一个活跃的节点
				const BevNode *oGetLastActiveNode() const
				{
					return mo_LastActiveNode;
				}

				//设置活跃的节点为当前节点，同时更新父亲节点的活跃节点
				void SetActiveNode(BevNode *_o_Node)
				{
					mo_LastActiveNode = mo_ActiveNode;
					mo_ActiveNode = _o_Node;
					if (mo_ParentNode != NULL)
						mo_ParentNode->SetActiveNode(_o_Node);
				}

				//获取当前节点的名字
				const char *GetDebugName() const
				{
					return mz_DebugName.c_str();
				}

			protected:
				//--------------------------------------------------------------
				// virtual function 这一块由子类自己去完成，行为节点无需实现_DoEvaluate，需要自动返回true
				//--------------------------------------------------------------
				virtual bool _DoEvaluate(const BevNodeInputParam &input)
				{
					return true;
				}
				virtual void _DoTransition(const BevNodeInputParam &input)
				{
				}
				virtual BevRunningStatus _DoTick(const BevNodeInputParam &input, BevNodeOutputParam &output)
				{
					return k_BRS_Finish;
				}

			protected:
				//设置父亲节点
				void _SetParentNode(BevNode *_o_ParentNode)
				{
					mo_ParentNode = _o_ParentNode;
				}

				//检查索引是否越界
				bool _bCheckIndex(int _ui_Index) const
				{
					return _ui_Index >= 0 && _ui_Index < mul_ChildNodeCount;
				}

			protected:
				//用来存放孩子节点
				BevNode *mao_ChildNodeList[k_BLimited_MaxChildNodeCnt];
				//记录孩子节点的数量
				int mul_ChildNodeCount;
				//记录父亲节点
				BevNode *mo_ParentNode;
				//记录当前活跃的节点
				BevNode *mo_ActiveNode;
				//记录上一个活跃的节点
				BevNode *mo_LastActiveNode;
				//前提条件
				BevNodePrecondition *mo_NodePrecondition;
				//名称
				std::string mz_DebugName;
			};
		
			//有优先级的选择类型控制节点
			class BevNodePrioritySelector : public BevNode
			{
			public:

				//传入父亲节点和前提条件，会调用基类的构造函数。将索引置都为16，即越界，为空
				BevNodePrioritySelector(BevNode *_o_ParentNode, BevNodePrecondition *_o_NodePrecondition = NULL)
					: BevNode(_o_ParentNode, _o_NodePrecondition), mui_LastSelectIndex(k_BLimited_InvalidChildNodeIndex), mui_CurrentSelectIndex(k_BLimited_InvalidChildNodeIndex)
				{
				}
				
				//控制节点做评估的地方，会一直往下递推下去，直到找到第一个满足所有条件的行为节点。
				virtual bool _DoEvaluate(const BevNodeInputParam &input);

				//先检查越界，再切换到上一个行为，直到切换到没有上一个行为的节点
				virtual void _DoTransition(const BevNodeInputParam &input);

				//更新函数
				virtual BevRunningStatus _DoTick(const BevNodeInputParam &input, BevNodeOutputParam &output);

			protected:
				//当前选择的索引
				int mui_CurrentSelectIndex;
				//上一个选择的索引
				int mui_LastSelectIndex;
			};

			class BevNodeNonePrioritySelector : public BevNodePrioritySelector
			{
			public:
				BevNodeNonePrioritySelector(BevNode *_o_ParentNode, BevNodePrecondition *_o_NodePrecondition = NULL)
					: BevNodePrioritySelector(_o_ParentNode, _o_NodePrecondition)
				{
				}
				virtual bool _DoEvaluate(const BevNodeInputParam &input);
			};

			class BevNodeSequence : public BevNode
			{
			public:
				BevNodeSequence(BevNode *_o_ParentNode, BevNodePrecondition *_o_NodePrecondition = NULL)
					: BevNode(_o_ParentNode, _o_NodePrecondition), mui_CurrentNodeIndex(k_BLimited_InvalidChildNodeIndex)
				{
				}
				virtual bool _DoEvaluate(const BevNodeInputParam &input);
				virtual void _DoTransition(const BevNodeInputParam &input);
				virtual BevRunningStatus _DoTick(const BevNodeInputParam &input, BevNodeOutputParam &output);

			private:
				int mui_CurrentNodeIndex;
			};

			class BevNodeParallel : public BevNode
			{
			public:
				BevNodeParallel(BevNode *_o_ParentNode, BevNodePrecondition *_o_NodePrecondition = NULL)
					: BevNode(_o_ParentNode, _o_NodePrecondition), me_FinishCondition(k_PFC_OR)
				{
					for (unsigned int i = 0; i < k_BLimited_MaxChildNodeCnt; ++i)
						mab_ChildNodeStatus[i] = k_BRS_Executing;
				}
				virtual bool _DoEvaluate(const BevNodeInputParam &input);
				virtual void _DoTransition(const BevNodeInputParam &input);
				virtual BevRunningStatus _DoTick(const BevNodeInputParam &input, BevNodeOutputParam &output);

				BevNodeParallel &SetFinishCondition(E_ParallelFinishCondition _e_Condition);

			private:
				E_ParallelFinishCondition me_FinishCondition;
				BevRunningStatus mab_ChildNodeStatus[k_BLimited_MaxChildNodeCnt];
			};

			class BevNodeLoop : public BevNode
			{
			public:
				static const int kInfiniteLoop = -1;

			public:
				BevNodeLoop(BevNode *_o_ParentNode, BevNodePrecondition *_o_NodePrecondition = NULL, int _i_LoopCnt = kInfiniteLoop)
					: BevNode(_o_ParentNode, _o_NodePrecondition), mi_LoopCount(_i_LoopCnt), mi_CurrentCount(0)
				{
				}
				virtual bool _DoEvaluate(const BevNodeInputParam &input);
				virtual void _DoTransition(const BevNodeInputParam &input);
				virtual BevRunningStatus _DoTick(const BevNodeInputParam &input, BevNodeOutputParam &output);

			private:
				int mi_LoopCount;
				int mi_CurrentCount;
			};

			//行为节点的基类，所有的行为节点都要继承与这个类
			class BevNodeTerminal : public BevNode
			{
			public:
				BevNodeTerminal(BevNode *_o_ParentNode, BevNodePrecondition *_o_NodePrecondition = NULL)
					: BevNode(_o_ParentNode, _o_NodePrecondition), me_Status(k_TNS_Ready), mb_NeedExit(false)
				{}

				//行为节点的转移就是把节点状态初始化了
				virtual void _DoTransition(const BevNodeInputParam &input);
				
				//更新函数
				virtual BevRunningStatus _DoTick(const BevNodeInputParam &input, BevNodeOutputParam &output);
			protected:
				//进入这个类时会调用的接口，可用于初始化一些东西
				virtual void _DoEnter(const BevNodeInputParam &input) {}
				//执行行为的函数，可以在这里对传入的数据进行判断。
				virtual BevRunningStatus _DoExecute(const BevNodeInputParam &input, BevNodeOutputParam &output) { return k_BRS_Finish; }
				//退出类时会调用的接口（转移控制权或者结束行为时会调用），
				virtual void _DoExit(const BevNodeInputParam &input, BevRunningStatus _ui_ExitID) {}

			private:
				E_TerminalNodeStaus me_Status;
				bool mb_NeedExit;
			};

			class BevNodeFactory
			{
			public:
				static BevNode &oCreateParallelNode(BevNode *_o_Parent, E_ParallelFinishCondition _e_Condition, const char *_debugName)
				{
					BevNodeParallel *pReturn = new BevNodeParallel(_o_Parent);
					pReturn->SetFinishCondition(_e_Condition);
					oCreateNodeCommon(pReturn, _o_Parent, _debugName);
					return (*pReturn);
				}
				static BevNode &oCreatePrioritySelectorNode(BevNode *_o_Parent, const char *_debugName)
				{
					BevNodePrioritySelector *pReturn = new BevNodePrioritySelector(_o_Parent);
					oCreateNodeCommon(pReturn, _o_Parent, _debugName);
					return (*pReturn);
				}
				static BevNode &oCreateNonePrioritySelectorNode(BevNode *_o_Parent, const char *_debugName)
				{
					BevNodeNonePrioritySelector *pReturn = new BevNodeNonePrioritySelector(_o_Parent);
					oCreateNodeCommon(pReturn, _o_Parent, _debugName);
					return (*pReturn);
				}
				static BevNode &oCreateSequenceNode(BevNode *_o_Parent, const char *_debugName)
				{
					BevNodeSequence *pReturn = new BevNodeSequence(_o_Parent);
					oCreateNodeCommon(pReturn, _o_Parent, _debugName);
					return (*pReturn);
				}
				static BevNode &oCreateLoopNode(BevNode *_o_Parent, const char *_debugName, int _i_LoopCount)
				{
					BevNodeLoop *pReturn = new BevNodeLoop(_o_Parent, NULL, _i_LoopCount);
					oCreateNodeCommon(pReturn, _o_Parent, _debugName);
					return (*pReturn);
				}
				template <typename T>
				static BevNode &oCreateTeminalNode(BevNode *_o_Parent, const char *_debugName)
				{
					BevNodeTerminal *pReturn = new T(_o_Parent);
					oCreateNodeCommon(pReturn, _o_Parent, _debugName);
					return (*pReturn);
				}

			private:
				static void oCreateNodeCommon(BevNode *_o_Me, BevNode *_o_Parent, const char *_debugName)
				{
					if (_o_Parent)
						_o_Parent->AddChildNode(_o_Me);
					_o_Me->SetDebugName(_debugName);
				}
			};
		}
	}
}

#endif
