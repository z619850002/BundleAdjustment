#ifndef OPTIMIZER_H_
#define OPTIMIZER_H_

#include "../error/base_error.h"
#include "../variable/base_variables.h"

#include <map>
#include <set>
using namespace std;


class Block{
public:
	Block(	int nStartPos, int nDimension)
		: m_nStartPos(nStartPos), m_nDimension(nDimension)
	{}
	int m_nStartPos;
	int m_nDimension;

};

class Block2D{

public:
	Block2D( int nStartRow, int nStartCol, int nRows, int nCols)
		: m_nStartRow(nStartRow), m_nStartCol(nStartCol), m_nRows(nRows), m_nCols(nCols)
	{}

	int m_nStartRow;
	int m_nStartCol;
	int m_nRows;
	int m_nCols;
	
};


class Optimizer
{
public:
	Optimizer(){

	}

	bool AddEdge(BaseErrorTerm * pErrorTerm){
		int nIndex = pErrorTerm->GetID();
		this->m_mEdges[nIndex] = pErrorTerm;
		this->m_mEdgeIndices[pErrorTerm] = nIndex;
	}


	bool AddVertex(OptimizedVariable * pVariable){
		int nIndex = pVariable->GetID();
		this->m_mVertices[nIndex] = pVariable;
		this->m_mVertexIndices[pVariable] = nIndex;
	}

	inline BaseErrorTerm * GetEdge(int nID){
		return this->m_mEdges[nID];
	}

	inline OptimizedVariable * GetVertex(int nID){
		return this->m_mVertices[nID];
	}

	bool Optimize(int nIterations){
		for (int nIter = 0; nIter < nIterations; nIter++){
			cout << "Iteration: " << nIter << endl;
			//Firstly compute the jacobian matrix.
			this->GenerateJacobianMatrix();
			//Secondly compute the error.
			this->GenerateErrorVector();

			Eigen::MatrixXd mH = this->m_mJacobian.transpose() * this->m_mJacobian;
			Eigen::VectorXd mG = -this->m_mJacobian.transpose() * this->m_mError;

			Eigen::VectorXd mDelta = mH.inverse() * mG;

			//Update.
			this->Update(mDelta);

		}
		return true;
	}

	bool InitializeOptimziation(){
		this->ManageMatrixShape();
		return true;
	}



public:

	bool inline GenerateJacobianMatrix(){
		for (map<int, BaseErrorTerm *>::iterator pIterator = this->m_mEdges.begin(); 
				pIterator != this->m_mEdges.end(); 
				pIterator++){
			BaseErrorTerm * pErrorTerm = pIterator->second;
			pErrorTerm->ComputeJacobian();
			Block * pEdgeBlock = m_mEdgePositions[pErrorTerm];
			// for (int i=0;i<pErrorTerm->Get)
			vector<OptimizedVariable *> gVariables = pErrorTerm->GetVariables();
			vector<Eigen::MatrixXd> gJacobians = pErrorTerm->GetJacobians();
			for (int i=0;i<gVariables.size();i++){
				OptimizedVariable * pVariable = gVariables[i];
				Eigen::MatrixXd mJacobian = gJacobians[i];
				Block * pVertexBlock = m_mVertexPositions[pVariable];
				this->m_mJacobian.block(pEdgeBlock->m_nStartPos, 
										pVertexBlock->m_nStartPos,
										pEdgeBlock->m_nDimension,
										pVertexBlock->m_nDimension) = mJacobian;
			}
		}
		cout << "Jacobian is: " << endl << this->m_mJacobian << endl;
		return true;
	}

	//Compute the error.
	bool inline GenerateErrorVector(){
		for (map<int, BaseErrorTerm *>::iterator pIterator = this->m_mEdges.begin(); 
				pIterator != this->m_mEdges.end(); 
				pIterator++){
			BaseErrorTerm * pErrorTerm = pIterator->second;
			Eigen::VectorXd mError =  pErrorTerm->ComputeError();
			Block * pEdgeBlock = m_mEdgePositions[pErrorTerm];
			this->m_mError.segment(pEdgeBlock->m_nStartPos, pEdgeBlock->m_nDimension) = mError;
		}
		cout << "Error is: " << endl << this->m_mError << endl;
		return true;
	}


	bool inline Update(Eigen::VectorXd mDelta){
		for (	map<int, OptimizedVariable *>::iterator pIterator = this->m_mVertices.begin(); 
				pIterator != this->m_mVertices.end(); 
				pIterator++){
			OptimizedVariable * pVariable = pIterator->second;
			Block * pBlock = this->m_mVertexPositions[pVariable];
			Eigen::VectorXd mLocalDelta = mDelta.segment(pBlock->m_nStartPos, pBlock->m_nDimension);
			pVariable->Update(mLocalDelta);
		}	
		return true;
	}


	//Get the block of the jacobian of an error to a variable.
	inline Block2D * GetBlock(BaseErrorTerm * pErrorTerm, OptimizedVariable * pVariable){
		Block * pEdgeBlock = m_mEdgePositions[pErrorTerm];
		Block * pVertexBlock = m_mVertexPositions[pVariable];
		return new Block2D(	pEdgeBlock->m_nStartPos, 
							pVertexBlock->m_nStartPos,
							pEdgeBlock->m_nDimension,
							pVertexBlock->m_nDimension);
	}


	bool ManageMatrixShape(){

		//Calculate the dimension of all errors.
		int nTotalErrorDimension = 0;
		for (	map<int, BaseErrorTerm *>::iterator pIterator = this->m_mEdges.begin(); 
				pIterator != this->m_mEdges.end(); 
				pIterator++){
			BaseErrorTerm * pErrorTerm = pIterator->second;
			Block * pBlock = new Block(nTotalErrorDimension ,pErrorTerm->GetErrorDimension());
			this->m_mEdgePositions[pErrorTerm] = pBlock;
			nTotalErrorDimension += pErrorTerm->GetErrorDimension();
		}

		//Calculate the dimension of all variables.
		int nTotalVariableDimension = 0;
		for (	map<int, OptimizedVariable *>::iterator pIterator = this->m_mVertices.begin(); 
				pIterator != this->m_mVertices.end(); 
				pIterator++){
			OptimizedVariable * pVariable = pIterator->second;
			Block * pBlock = new Block(nTotalVariableDimension, pVariable->GetDimension());
			this->m_mVertexPositions[pVariable] = pBlock;
			nTotalVariableDimension += pVariable->GetDimension();
		}	

		this->m_mJacobian = Eigen::MatrixXd(nTotalErrorDimension, nTotalVariableDimension);	
		this->m_mError = Eigen::VectorXd(nTotalErrorDimension);
	}


	map<int, BaseErrorTerm *> m_mEdges;
	map<int, OptimizedVariable *> m_mVertices;

	//Use vertex and edge to find id.
	map<BaseErrorTerm*, int> m_mEdgeIndices;
	map<OptimizedVariable*, int> m_mVertexIndices;

	//Use vertex and edge to find the position in the Jacobian.
	map<BaseErrorTerm*, Block *> m_mEdgePositions;
	map<OptimizedVariable*, Block *> m_mVertexPositions;

	Eigen::MatrixXd m_mJacobian;
	Eigen::VectorXd m_mError;

};



#endif