/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#ifndef EQUIVALENCE_RELATIONS_H_
#define EQUIVALENCE_RELATIONS_H_

#include <boost/shared_ptr.hpp>

namespace teb_local_planner
{

/**
 * @class EquivalenceClass
 * @brief Abstract class that defines an interface for computing and comparing equivalence classes
 *
 * Equivalence relations are utilized in order to test if two trajectories are belonging to the same
 * equivalence class w.r.t. the current obstacle configurations. A common equivalence relation is
 * the concept of homotopy classes. All trajectories belonging to the same homotopy class
 * can CONTINUOUSLY be deformed into each other without intersecting any obstacle. Hence they likely
 * share the same local minimum after invoking (local) trajectory optimization. A weaker equivalence relation
 * is defined by the concept of homology classes (e.g. refer to HSignature).
 *
 * Each EquivalenceClass object (or subclass) stores a candidate value which might be compared to another EquivalenceClass object.
 *
 * @remarks Currently, the computeEquivalenceClass method is not available in the generic interface EquivalenceClass.
 *          Call the "compute"-methods directly on the subclass.
 */
class EquivalenceClass
{
public:

   /**
    * @brief Default constructor
    */
   EquivalenceClass() {}

   /**
    * @brief virtual destructor
    */
   virtual ~EquivalenceClass() {}

   /**
    * @brief Check if two candidate classes are equivalent
    * @param other The other equivalence class to test with
    */
   virtual bool isEqual(const EquivalenceClass& other) const = 0;

   /**
    * @brief Check if the equivalence value is detected correctly
    * @return Returns false, if the equivalence class detection failed, e.g. if nan- or inf values occur.
    */
   virtual bool isValid() const = 0;

   /**
    * @brief Check if the trajectory is non-looping around an obstacle
    * @return Returns false, if the trajectory loops around an obstacle
    */
   virtual bool isReasonable() const = 0;

};

using EquivalenceClassPtr = boost::shared_ptr<EquivalenceClass>;


} // namespace teb_local_planner


#endif /* EQUIVALENCE_RELATIONS_H_ */
